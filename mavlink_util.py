"""mavlink_util.py — Konversi & utilitas MAVLink untuk bench servo QGC.

Isolasi konstanta protokol dan fungsi konversi agar bisa diuji unit
(tests/test_mavlink_util.py). Nilai berasal dari spesifikasi MAVLink
pesan MANUAL_CONTROL:

  - x, y, r : int16, rentang [-127, 127]  (rush/pitch, roll, yaw)
  - z       : uint16, rentang [0, 1000], netral = 500 (throttle)

Untuk kapal permukaan (ASV) hanya dua derajat kebebasan yang dipakai:
  - v (surge) dari throttle z  -> maju / mundur
  - w (yaw)   dari stick r     -> belok kiri / kanan
"""

Z_MIN = 0.0
Z_MAX = 1000.0
Z_NEUTRAL = (Z_MIN + Z_MAX) / 2.0   # 500 = stik throttle di tengah
R_FULL = 127.0                      # deviasi yaw stick penuh


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

    x dan y (roll/pitch) tidak relevan untuk kapal permukaan -> 0.
    """
    z, r = vw_to_manual(v, w)
    master.mav.manual_control_send(target_system, 0, 0, z, r, 0)


def neutral_manual_send(master, target_system=1):
    """Kirim MANUAL_CONTROL netral (v=0, w=0) — dipakai failsafe/berhenti."""
    z, r = neutral_values()
    master.mav.manual_control_send(target_system, 0, 0, z, r, 0)


def detect_serial_uart():
    """Path kandidat port serial Pixhawk di Linux.

    Urutan prioritas:
      1. /dev/serial/by-id/*  (symlink stabil, umumnya berisi "pixhawk")
      2. /dev/ttyACM*         (USB CDC, banyak dipakai Pixhawk)
      3. /dev/ttyUSB*         (adaptor USB-UART FTDI/CP210x)
    """
    import glob

    candidates = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        candidates += sorted(glob.glob(pattern))
    return candidates