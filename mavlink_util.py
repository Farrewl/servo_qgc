Z_MIN = 0.0
Z_MAX = 1000.0
Z_NEUTRAL = (Z_MIN + Z_MAX) / 2.0   # 500 = stik throttle di tengah
R_FULL = 127.0                      # deviasi yaw stick penuh

# VID:PID USB yang umum dipakai Pixhawk / PX4 FMU / STM32 VCP.
PIXHAWK_IDS = frozenset({
    (0x26AC, 0x0011),  # PX4 FMU v2/v3
    (0x26AC, 0x0012),  # PX4 FMU v4/v5/v6 (termasuk Pixhawk 6C)
    (0x0483, 0x5740),  # STM32 Virtual COM Port
})

_PORT_KEYWORDS = ("px4", "pixhawk", "fmu", "stm32")


def clamp1(value):
    """Kunci nilai ke rentang [-1.0, 1.0]."""
    if value < -1.0:
        return -1.0
    if value > 1.0:
        return 1.0
    return value


def manual_to_vw(msg):
    """Objek pesan MAVLink MANUAL_CONTROL -> tuple (v, w) di [-1, 1].

    z di luar rentang protokol -> dianggap netral (pengaman: nilai liar
    jangan pernah diterjemahkan menjadi gas penuh).
    """
    try:
        z = float(getattr(msg, "z", Z_NEUTRAL))
        r = float(getattr(msg, "r", 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0
    if z < Z_MIN or z > Z_MAX:
        return 0.0, 0.0
    v = (z - Z_NEUTRAL) / (Z_MAX - Z_NEUTRAL)
    w = r / R_FULL
    return clamp1(v), clamp1(w)


def vw_to_manual(v, w):
    """(v, w) di [-1, 1] -> nilai mentah (z, r) pesan MANUAL_CONTROL."""
    v = clamp1(v)
    w = clamp1(w)
    z = int(round(v * (Z_MAX - Z_NEUTRAL) + Z_NEUTRAL))
    r = int(round(w * R_FULL))
    return z, r


def neutral_values():
    """Nilai MANUAL_CONTROL netral: throttle tengah, yaw lurus."""
    return int(Z_NEUTRAL), 0


def manual_send(master, v, w, target_system=1):
    """Kirim MANUAL_CONTROL (v, w) ke Pixhawk lewat koneksi `master`.

    Untuk tes thruster: v = surge (maju/mundur), w = 0.
    """
    z, r = vw_to_manual(v, w)
    master.mav.manual_control_send(target_system, 0, 0, z, r, 0)


def neutral_manual_send(master, target_system=1):
    """Kirim MANUAL_CONTROL netral (v=0, w=0) — dipakai failsafe/berhenti."""
    z, r = neutral_values()
    master.mav.manual_control_send(target_system, 0, 0, z, r, 0)


def is_port_busy_error(exc):
    """True bila error mengindikasikan port dikunci aplikasi lain.

    Di Windows ini umum terjadi kalau link serial QGC ke Pixhawk masih
    nyambung saat skrip Python mencoba buka COM yang sama.
    """
    text = str(exc).lower()
    return any(k in text for k in ("permission", "denied", "busy",
                                  "access", "lock"))


def _looks_like_pixhawk(info):
    """Cek info port pyserial: cocok VID/PID atau nama produk."""
    try:
        if (info.vid, info.pid) in PIXHAWK_IDS:
            return True
    except (AttributeError, TypeError):
        pass
    desc = " ".join(str(getattr(info, name, "") or "")
                    for name in ("description", "product", "manufacturer"))
    return any(k in desc.lower() for k in _PORT_KEYWORDS)


def detect_serial_uart():
    """Daftar kandidat port serial Pixhawk (Windows + Linux).

    Urutan prioritas:
      1. Port yang VID/PID/namanya cocok Pixhawk/PX4 (mis. COM3).
      2. Bila tak ada yang cocok: di Windows kembalikan semua COM,
         di Linux pakai pola /dev klasik (/dev/ttyACM*, /dev/ttyUSB*).
    """
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
    except Exception:
        ports = []
    pix = sorted(p.device for p in ports if _looks_like_pixhawk(p))
    if pix:
        return pix
    if ports:
        import sys
        if sys.platform.startswith("win"):
            return sorted(p.device for p in ports)
    import glob

    candidates = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        candidates += sorted(glob.glob(pattern))
    return candidates


def connect_px(serial_path, baud, log=None):
    """Konek serial ke Pixhawk, auto-detect bila path kosong.

    Dipakai bersama oleh sweep.py & offboard_thrust.py. Return koneksi
    pymavlink atau None (dengan pesan error yang jelas lebih dulu).
    """
    import logging as _logging

    log = log or _logging.getLogger("servo_qgc")
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
        from pymavlink import mavutil
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
