#!/usr/bin/env python3
"""demo_loop.py — Latihan kendali TANPA hardware & TANPA QGC.

Mensimulasikan apa yang dikirim QGC (MANUAL_CONTROL) dari stick, lalu
menampilkan (v, w) yang akan diteruskan ke Pixhawk — skala identik.

Tombol (terminal Linux raw):
    W / S  : surge maju / mundur    (v)
    A / D  : yaw kiri / kanan       (w)
    SPASI  : netral (v=0, w=0)
    Q      : keluar

Cocok untuk membiasakan arah stik sebelum menyolok servo asli.
"""

import sys
import termios
import tty

from mavlink_util import clamp1

STEP = 0.25          # besar langkah tiap penekanan tombol
DELTA_WASD = "wasd"  # tombol aktif (lowercase)

# Peta tombol -> (delta v, delta w)
KEY_MAP = {
    "w": (+STEP, 0.0),
    "s": (-STEP, 0.0),
    "a": (0.0, -STEP),
    "d": (0.0, +STEP),
    " ": (None, None),   # reset ke netral
}


def getch():
    """Baca 1 karakter terminal dalam mode raw (Linux/termios)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def clamp_display(v, w):
    """Varian clamp yang pasti mengembalikan float (test-friendly)."""
    return clamp1(v), clamp1(w)


def main():
    v, w = 0.0, 0.0
    print("Demo kendali keyboard (tanpa hardware).")
    print("  W/S = surge, A/D = yaw, SPASI = netral, Q = keluar")
    print()
    while True:
        ch = getch().lower()
        if ch == "q":
            print("\nKeluar dari demo.")
            break
        if ch in KEY_MAP:
            dv, dw = KEY_MAP[ch]
            if dv is None:
                v, w = 0.0, 0.0
            else:
                v = clamp1(v + dv)
                w = clamp1(w + dw)
        # Tampilan "panel" mini tiap penekanan.
        bar_v = "|" * int(abs(v) * 20)
        bar_w = "|" * int(abs(w) * 20)
        sys.stdout.write("\r[surge v=%+.2f %-20s] [yaw w=%+.2f %-20s]  "
                         "lanjutkan: W/S/A/D SPASI Q  " % (v, bar_v, w, bar_w))
        sys.stdout.flush()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())