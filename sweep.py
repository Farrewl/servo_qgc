import argparse
import logging
import signal
import sys
import time

from pymavlink import mavutil

from mavlink_util import (detect_serial_uart, is_port_busy_error,
                          manual_send, neutral_manual_send)

DEFAULT_BAUD = 115200
DEFAULT_HOLD_S = 2.0         # lama tahan tiap posisi (biar kelihatan)
SETTLE_S = 0.05              # jeda antar perintah (biar tidak nyeret)

# Siklus uji thruster: (v surge, label). v=+1 maju, v=-1 mundur.
SWEEP_STEPS = (
    (0.0, "netral"),
    (1.0, "MAJU penuh"),
    (0.0, "netral"),
    (-1.0, "MUNDUR penuh"),
    (0.0, "netral"),
)

log = logging.getLogger("servo_qgc")


def connect_px(serial_path, baud):
    """Konek serial ke Pixhawk, auto-detect bila path kosong."""
    if serial_path:
        path = serial_path
    else:
        candidates = detect_serial_uart()
        if candidates:
            log.info("Port kandidat: %s", ", ".join(candidates))
        path = candidates[0] if candidates else ""
    if not path:
        log.error("Port serial tidak ditemukan. Cek Device Manager > "
                  "Ports (COM & LPT), colok USB Pixhawk lalu coba lagi.")
        return None
    try:
        px = mavutil.mavlink_connection(path, baud=baud)
        log.info("Pixhawk terhubung via %s @ %d baud", path, baud)
        return px
    except Exception as exc:
        if is_port_busy_error(exc):
            log.error("Port %s dipakai aplikasi lain — putuskan link serial "
                      "QGC ke Pixhawk dulu lalu coba lagi.", path)
        else:
            log.error("Koneksi serial gagal (%s): %s", path, exc)
        return None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial", default="",
                   help="port Pixhawk, mis. COM3 (Windows) atau "
                        "/dev/ttyACM0 (Linux). Kosong = auto-detect.")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="baud Pixhawk (USB=115200).")
    p.add_argument("--hold", type=float, default=DEFAULT_HOLD_S,
                   help="detik tahan tiap posisi (default %(default)s).")
    p.add_argument("--cycles", type=int, default=0,
                   help="jumlah siklus, 0 = ulang terus sampai ^C "
                        "(default %(default)s).")
    args = p.parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    px = connect_px(args.serial, args.baud)
    if px is None:
        return 1
    state = {"stop": False}
    signal.signal(signal.SIGINT, lambda *_: state.update(stop=True))
    try:
        signal.signal(signal.SIGTERM, lambda *_: state.update(stop=True))
    except (AttributeError, ValueError, OSError):
        pass  # SIGTERM tidak tersedia di semua platform

    if args.cycles <= 0:
        log.info("Siklus maju/mundur berulang (tahan %.1fs tiap posisi). "
                 "^C untuk berhenti.", args.hold)
    else:
        log.info("Jalankan %d siklus maju/mundur (tahan %.1fs tiap posisi).",
                 args.cycles, args.hold)
    done = 0
    try:
        while not state["stop"]:
            for v, label in SWEEP_STEPS:
                if state["stop"]:
                    break
                log.info(">>> v=%+.2f (%s) — amati arah putaran thruster",
                         v, label)
                manual_send(px, v, 0.0)
                time.sleep(SETTLE_S)
                time.sleep(max(0.0, args.hold - SETTLE_S))
            done += 1
            if args.cycles > 0 and done >= args.cycles:
                break
    finally:
        neutral_manual_send(px)
        time.sleep(0.1)
        px.close()
        log.info("Selesai — netral dikirim, thruster berhenti.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
