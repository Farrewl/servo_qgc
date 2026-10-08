"""offboard_thrust.py — Tes thruster MAJU/MUNDUR via mode OFFBOARD (PX4 v1.17).

Alur kerja (urutan ini WAJIB, sesuai aturan PX4 Offboard):

    1. Stream setpoint velocity 10 Hz dulu (proof-of-life, vx=0).
    2. Minta mode OFFBOARD via MAV_CMD_DO_SET_MODE.
    3. Arm (kecuali --no-arm).
    4. Kontrol vx dari keyboard; stream TIDAK PERNAH berhenti selama Offboard.

Kontrol keyboard (terminal aktif):
    W / S  : tambah / kurangi kecepatan maju (vx += step)
    X / SPASI : berhenti (vx = 0, stream tetap jalan)
    O      : coba masuk Offboard lagi (kalau kepental keluar)
    Q      : keluar aman (vx=0 -> MANUAL -> disarm -> tutup)

SYARAT DI PIXHAWK (set sekali via QGC > Parameters, lalu reboot):
    MAV_FWDEXTSP   = 1   (teruskan setpoint eksternal ke modul rover;
                          default 0 = "No offboard signal" selamanya)
    COM_ARM_WO_GPS = 1   (boleh arm tanpa GPS, untuk bench indoor)

PENTING:
  - Satu port COM hanya bisa dibuka SATU aplikasi: putuskan link SERIAL
    QGC ke Pixhawk sebelum menjalankan skrip ini.
  - Bench darat: LEPAS baling-baling thruster sebelum menjalankan!

PEMAKAIAN (PowerShell di NUC):
    python offboard_thrust.py --serial COM3
    python offboard_thrust.py --serial COM3 --no-arm   # cek mode tanpa gerak
    python offboard_thrust.py --serial COM3 --speed 1.0 --step 0.25
"""

import argparse
import logging
import sys
import threading
import time

from pymavlink import mavutil

from mavlink_util import clamp1, connect_px

SETPOINT_HZ = 10.0           # laju stream proof-of-life (syarat PX4: >= 2 Hz)
PRESTREAM_S = 3.0            # stream vx=0 sekian detik SEBELUM minta Offboard
ARM_COUNTDOWN_S = 3.0        # hitung mundur sebelum arm otomatis
MODE_TIMEOUT_S = 5.0         # tunggu Pixhawk pindah mode
ACK_TIMEOUT_S = 3.0          # tunggu COMMAND_ACK
DEFAULT_BAUD = 115200
DEFAULT_MAX_VX = 2.0         # batas vx (m/s, body-frame, + = maju)
DEFAULT_STEP_VX = 0.5        # langkah tiap tekan W/S (m/s)

PX4_MODE_MANUAL = 1
PX4_MODE_OFFBOARD = 6

log = logging.getLogger("servo_qgc")


def velocity_type_mask():
    """Type-mask velocity-only: posisi/accel/yaw diabaikan, vx+vy aktif."""
    m = mavutil.mavlink
    return (m.POSITION_TARGET_TYPEMASK_X_IGNORE
            | m.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)


def clamp_vx(vx, max_vx):
    """Batasi vx ke [-max_vx, +max_vx]."""
    span = abs(float(max_vx))
    return max(-span, min(span, float(vx)))


class KeyReader(object):
    """Baca 1 tombol tanpa Enter (Windows msvcrt / Linux termios)."""

    def __init__(self):
        self._win = sys.platform.startswith("win")
        self._old = None
        if self._win:
            import msvcrt
            self._msvcrt = msvcrt
        else:
            import termios
            import tty
            fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(fd)
            tty.setraw(fd)

    def close(self):
        if not self._win and self._old is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                              self._old)
            self._old = None

    def get(self):
        """Satu tombol lowercase, atau None bila tidak ada yang ditekan."""
        if self._win:
            if not self._msvcrt.kbhit():
                return None
            ch = self._msvcrt.getwch()
            if ch in ("\x03", "\x1a"):  # Ctrl+C / Ctrl+Z
                raise KeyboardInterrupt
            return ch.lower()
        import select
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        import os
        data = os.read(sys.stdin.fileno(), 1).decode("utf-8", "ignore")
        if not data:
            return None
        if ord(data) == 3:  # Ctrl+C
            raise KeyboardInterrupt
        return data.lower()


def wait_heartbeat(master, timeout):
    """Tunggu HEARTBEAT pertama; return pesannya atau None."""
    msg = master.wait_heartbeat(timeout=timeout)
    if msg is None:
        log.error("Tidak ada HEARTBEAT dari Pixhawk dalam %.0fs.", timeout)
        return None
    log.info("HEARTBEAT: sys=%d mode(custom=%d) %s",
             msg.get_srcSystem(), msg.custom_mode,
             "ARMED" if msg.base_mode
             & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARM")
    return msg


def wait_mode(master, custom_mode, timeout):
    """Tunggu custom_mode tampil di HEARTBEAT; True bila tercapai."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is not None and msg.custom_mode == custom_mode:
            return True
    return False


def send_ack(master, command, p1=0.0, p2=0.0, timeout=ACK_TIMEOUT_S):
    """Kirim COMMAND_LONG; True bila ACK = ACCEPTED."""
    sysid = master.target_system
    compid = master.target_component
    master.mav.command_long_send(sysid, compid, command, 0,
                                 p1, p2, 0, 0, 0, 0, 0)
    ack = master.recv_match(type="COMMAND_ACK", blocking=True,
                            timeout=timeout)
    ok = (ack is not None
          and ack.command == command
          and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    if not ok:
        log.warning("ACK %s: %s", command,
                    ("ditolak/timeout" if ack is None
                     else "result=%d" % ack.result))
    return ok


def request_offboard(master):
    """Minta mode OFFBOARD; True bila HEARTBEAT mengonfirmasi."""
    send_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
             float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
             float(PX4_MODE_OFFBOARD))
    if wait_mode(master, PX4_MODE_OFFBOARD, MODE_TIMEOUT_S):
        log.info("Mode OFFBOARD aktif.")
        return True
    log.error("Gagal masuk OFFBOARD. Cek: (1) stream setpoint jalan "
              "(skrip ini), (2) MAV_FWDEXTSP=1 + reboot, "
              "(3) estimasi posisi/EKF valid (tanpa GPS bisa ditolak).")
    return False


def preflight_report(master):
    """Tampilkan GPS + EKF sekilas (tidak fatal bila tidak ada datanya)."""
    m = mavutil.mavlink
    for msg_id in (m.MAVLINK_MSG_ID_GPS_RAW_INT,
                   m.MAVLINK_MSG_ID_EKF_STATUS_REPORT):
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                m.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(msg_id), 1000000, 0, 0, 0, 0, 0)
        except Exception as exc:
            log.debug("Minta interval msg %d gagal: %s", msg_id, exc)
    gps = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
    if gps is None:
        log.info("GPS: tidak terdeteksi (wajar untuk bench tanpa GPS).")
    else:
        log.info("GPS: fix=%d sats=%d", gps.fix_type,
                 gps.satellites_visible)
    ekf = master.recv_match(type="EKF_STATUS_REPORT", blocking=True,
                            timeout=2.0)
    if ekf is not None:
        log.info("EKF flags: 0x%x", ekf.flags)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial", default="",
                   help="port Pixhawk, mis. COM3 (Windows). "
                        "Kosong = auto-detect.")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="baud Pixhawk (USB=115200).")
    p.add_argument("--speed", type=float, default=DEFAULT_MAX_VX,
                   help="batas |vx| m/s (default %(default)s).")
    p.add_argument("--step", type=float, default=DEFAULT_STEP_VX,
                   help="langkah vx tiap tekan W/S (default %(default)s).")
    p.add_argument("--no-arm", action="store_true",
                   help="jangan arm: hanya stream + masuk Offboard "
                        "(verifikasi mode tanpa thruster gerak).")
    args = p.parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    master = connect_px(args.serial, args.baud)
    if master is None:
        return 1
    if wait_heartbeat(master, timeout=10) is None:
        master.close()
        return 1
    preflight_report(master)

    t0 = time.monotonic()
    state = {"vx": 0.0, "stop": False, "offboard": False}
    lock = threading.Lock()

    def stream_loop():
        mask = velocity_type_mask()
        period = 1.0 / SETPOINT_HZ
        while not state["stop"]:
            with lock:
                vx = state["vx"]
            master.mav.set_position_target_local_ned_send(
                int((time.monotonic() - t0) * 1000),
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED, mask,
                0, 0, 0, vx, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(period)

    streamer = threading.Thread(target=stream_loop, daemon=True)

    keys = KeyReader()
    try:
        log.info("Stream setpoint %.0f Hz (vx=0) selama %.0fs dulu...",
                 SETPOINT_HZ, PRESTREAM_S)
        streamer.start()
        time.sleep(PRESTREAM_S)

        if not request_offboard(master):
            return 1
        state["offboard"] = True

        if args.no_arm:
            log.info("--no-arm: tidak arm. Tekan O/W/S/X untuk cek, "
                     "Q untuk keluar.")
        else:
            log.info("ARM dalam %.0fs — LEPAS BALING-BALING! "
                     "(Ctrl+C untuk batal)", ARM_COUNTDOWN_S)
            try:
                time.sleep(ARM_COUNTDOWN_S)
            except KeyboardInterrupt:
                log.info("Arm dibatalkan.")
                return 1
            if not send_ack(master,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            1.0):
                log.error("ARM ditolak (cek COM_ARM_WO_GPS=1 bila tanpa "
                          "GPS, dan pre-arm check di QGC).")
                return 1
            log.info("ARMED. W/S = maju/mundur, X/SPASI = berhenti, "
                     "O = offboard lagi, Q = keluar.")

        last_show = 0.0
        while True:
            try:
                ch = keys.get()
            except KeyboardInterrupt:
                break
            if ch is not None:
                with lock:
                    if ch in ("w", "s"):
                        dv = args.step if ch == "w" else -args.step
                        state["vx"] = clamp_vx(state["vx"] + dv,
                                               args.speed)
                    elif ch in ("x", " "):
                        state["vx"] = 0.0
                    elif ch == "o":
                        request_offboard(master)
                    elif ch == "q":
                        break
                    vx_now = state["vx"]
                log.info("vx=%+.2f m/s (W/S gas, X diam, O offboard, Q "
                         "keluar)", vx_now)
            now = time.monotonic()
            if now - last_show >= 2.0:
                last_show = now
                hb = master.recv_match(type="HEARTBEAT", blocking=False)
                if hb is not None and hb.custom_mode != PX4_MODE_OFFBOARD:
                    log.warning("Pixhawk keluar OFFBOARD (custom=%d) — "
                                "stream tetap jalan, tekan O untuk masuk "
                                "lagi.", hb.custom_mode)
                with lock:
                    vx_now = state["vx"]
                log.info("... vx=%+.2f m/s, mode %s", vx_now,
                         "OFFBOARD" if state["offboard"] else "?")
            time.sleep(0.05)
    finally:
        log.info("Berhenti aman: vx=0 -> MANUAL -> disarm.")
        with lock:
            state["vx"] = 0.0
        time.sleep(0.3)
        send_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                 float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                 float(PX4_MODE_MANUAL))
        send_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
        state["stop"] = True
        streamer.join(timeout=1.0)
        keys.close()
        master.close()
        log.info("Selesai.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
