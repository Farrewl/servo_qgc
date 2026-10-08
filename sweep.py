#!/usr/bin/env python3
"""sweep.py — Tes gerak servo pelan (kiri-kanan) tanpa QGC.

Kirim MANUAL_CONTROL langsung ke Pixhawk via serial dengan siklus:

    netral -> kanan penuh -> netral -> kiri penuh -> netral

setiap langkah ditahan selama SWEEP_HOLD_S detik supaya bisa diamati
arah & batas kerjanya. Saat keluar (^C) netral dikirim dulu.

ASUMSI:
  - Pixhawk dalam mode MANUAL (set via QGC atau switch RC).
  - Servo steering colok pada output Pixhawk yang dipetakan ke channel
    yaw; thrust (throttle) dibiarkan netral (v = 0).
  - Singkirkan baling-baling / benda dekat servo sebelum menjalankan!

PEMAKAIAN:
    python3 sweep.py [--serial /dev/ttyACM0] [--baud 115200]
"""

import argparse
import logging
import signal
import sys
import time

from pymavlink import mavutil

from mavlink_util import detect_serial_uart, neutral_manual_send, manual_send

SWEEP_HOLD_S = 2.0          # lama tahan tiap posisi (biar kelihatan)
RETRY_INTERVAL_S = 2.0      # coba ulang koneksi serial bila gagal
SETTLE_S = 0.05             # jeda antar perintah (biar tidak nyeret)
DEFAULT_BAUD = 115200

# Urutan (v, w) — w berubah penuh, v tetap netral.
SWEEP_STEPS = [(0.0, 0.0), (0.0, 1.0), (0.0, 0.0),
               (0.0, -1.0), (0.0, 0.0)]

log = logging.getLogger("servo_qgc")


def connect_px(serial_path, baud):
    """Konek serial ke Pixhawk, auto-detect bila path kosong."""
    if serial_path:
        path = serial_path
    else:
        candidates = detect_serial_uart()
        path = candidates[0] if candidates else ""
    if not path:
        log.error("Port serial tidak ditemukan. Colok Pixhawk lalu coba lagi.")
        return None
    try:
        px = mavutil.mavlink_connection(path, baud=baud)
        log.info("Pixhawk terhubung via %s @ %d baud", path, baud)
        return px
    except Exception as exc:
        log.error("Koneksi serial gagal (%s): %s", path, exc)
        return None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial", default="", help="port Pixhawk (auto-detect "
                                                 "bila kosong).")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="baud Pixhawk (USB=115200, TELEM2 sering 57600).")
    args = p.parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    px = connect_px(args.serial, args.baud)
    if px is None:
        return 1
    state = {"stop": False}
    signal.signal(signal.SIGINT, lambda *_: state.update(stop=True))
    signal.signal(signal.SIGTERM, lambda *_: state.update(stop=True))

    log.info("Mulai sweep servo (%d posisi, tahan %.1fs tiap posisi). "
             "^C untuk berhenti.", len(SWEEP_STEPS), SWEEP_HOLD_S)
    try:
        while not state["stop"]:
            for v, w in SWEEP_STEPS:
                if state["stop"]:
                    break
                label = {0.0: "netral", 1.0: "KANAN penuh",
                         -1.0: "KIRI penuh"}.get(w, w)
                log.info(">>> v=%+.2f w=%+.2f (%s) — amati arah servo",
                         v, w, label)
                manual_send(px, v, w)
                time.sleep(SETTLE_S)
                time.sleep(SWEEP_HOLD_S - SETTLE_S)
    finally:
        neutral_manual_send(px)
        time.sleep(0.1)
        px.close()
        log.info("Selesai — netral dikirim, koneksi ditutup.")
    return 0


if __name__ == "__main__":
    sys.exit(main())