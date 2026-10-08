import argparse
import logging
import signal
import sys
import time

from pymavlink import mavutil

from mavlink_util import (detect_serial_uart, is_port_busy_error,
                          neutral_manual_send)

# --- Konstanta perilaku (bukan angka ajaib) ---
GCS_TIMEOUT_S = 1.5        # tanpa paket QGC selama ini -> dianggap putus
POLL_SLEEP_S = 0.005       # jeda tiap putaran poll (loop ~200 Hz)
RETRY_INTERVAL_S = 2.0     # coba ulang koneksi serial bila gagal
NEUTRAL_PERIOD_S = 0.2     # interval kirim netral saat failsafe
LOG_INTERVAL_S = 1.0       # interval log ringkasan
WARN_INTERVAL_S = 5.0      # interval peringatan failsafe (jangan spamming)
DEFAULT_GCS = "udpout:127.0.0.1:14550"
DEFAULT_BAUD = 115200

log = logging.getLogger("servo_qgc")


# MAV_MODE_FLAG — konstanta sudah disediakan pymavlink, dipakai langsung
# supaya tidak ada angka ajaib.
_FLAG_BITS = (
    ("MANUAL", mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
    ("STABILIZE", mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED),
    ("GUIDED", mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED),
    ("AUTO", mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED),
    ("TEST", mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED),
    ("CUSTOM", mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
)


def describe_mode(heartbeat):
    """Buat teks singkat mode Pixhawk dari pesan HEARTBEAT."""
    parts = [name for name, bit in _FLAG_BITS if heartbeat.base_mode & bit]
    armed = (heartbeat.base_mode &
             mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    mode = "+".join(parts) if parts else "?"
    return "[%s] custom=%d %s" % (mode, heartbeat.custom_mode,
                                  "ARMED" if armed else "DISARM")


class Router(object):
    """Jembatan QGC <-> Pixhawk + failsafe netral (tanpa monitor servo)."""

    def __init__(self, serial_path, baud, gcs_addr):
        self._serial_path = serial_path
        self._baud = baud
        self._gcs_addr = gcs_addr
        self.px = None
        self.gcs = None
        self._stopping = False
        self._retry_at = 0.0
        self._last_gcs_rx = None          # monotonic paket QGC terakhir
        self._last_neutral = None
        self._last_log = 0.0
        self._last_warn = 0.0
        self._mode_txt = "?"
        # Penghitung untuk log ringkasan/diagnosis.
        self.n_gcs = 0
        self.n_px = 0

    # ---------------------------------------------------------- koneksi
    def _ensure_px(self):
        """Hubungkan ke Pixhawk (serial/USB) secara non-blocking + retry."""
        if self.px is not None:
            return True
        now = time.monotonic()
        if now < self._retry_at:
            return False
        self._retry_at = now + RETRY_INTERVAL_S
        path = self._serial_path
        if not path:
            candidates = detect_serial_uart()
            if not candidates:
                log.warning("Belum ada port serial terdeteksi. "
                            "Colok USB Pixhawk lalu tunggu...")
                return False
            if len(candidates) > 1:
                log.info("Port kandidat: %s (pakai %s)",
                         ", ".join(candidates), candidates[0])
            path = candidates[0]
            log.info("Port auto-detect: %s", path)
        try:
            self.px = mavutil.mavlink_connection(path, baud=self._baud)
            log.info("Pixhawk terhubung via %s @ %d baud", path, self._baud)
            return True
        except Exception as exc:
            if is_port_busy_error(exc):
                log.warning("Port %s dipakai aplikasi lain (biasanya link "
                            "serial QGC masih nyambung). Putuskan link "
                            "serial di QGC dulu.", path)
            else:
                log.warning("Koneksi serial gagal (%s): %s", path, exc)
            self.px = None
            return False

    def _ensure_gcs(self):
        """Buka link UDP ke QGC (udpout: alamat QGC)."""
        if self.gcs is not None:
            return True
        try:
            self.gcs = mavutil.mavlink_connection(self._gcs_addr)
            log.info("Link GCS aktif: %s (pastikan QGC listen UDP 14550)",
                     self._gcs_addr)
            return True
        except Exception as exc:
            log.error("Gagal buka link GCS %s: %s", self._gcs_addr, exc)
            self.gcs = None
            return False

    # ---------------------------------------------------------- routing
    def _forward(self, src, dst, from_gcs):
        """Drain semua paket dari `src`, teruskan raw ke `dst` + pantau.

        from_gcs=True  : paket QGC->Pixhawk (catat waktu terima utk failsafe).
        from_gcs=False : paket Pixhawk->QGC (HEARTBEAT dipantau utk mode).
        """
        while True:
            msg = src.recv_match(blocking=False)
            if msg is None:
                break
            if from_gcs:
                self.n_gcs += 1
                self._last_gcs_rx = time.monotonic()
            else:
                self.n_px += 1
                if msg.get_type() == "HEARTBEAT":
                    self._mode_txt = describe_mode(msg)
            buf = msg.get_msgbuf()
            if buf and dst is not None:
                try:
                    dst.write(buf)
                except Exception as exc:
                    log.debug("Forward gagal: %s", exc)

    def _failsafe_neutral(self):
        """Kalau QGC basi > GCS_TIMEOUT_S -> kirim netral periodik."""
        now = time.monotonic()
        fresh = (self._last_gcs_rx is not None and
                 now - self._last_gcs_rx <= GCS_TIMEOUT_S)
        if fresh:
            return
        if self._last_neutral is None or \
                now - self._last_neutral >= NEUTRAL_PERIOD_S:
            self._last_neutral = now
            if self.px is not None:
                neutral_manual_send(self.px)
            if now - self._last_warn >= WARN_INTERVAL_S:
                self._last_warn = now
                log.warning("QGC tidak mengirim >%.1fs -> NETRAL dikirim "
                            "ke Pixhawk (thruster berhenti)", GCS_TIMEOUT_S)

    def _periodic_log(self):
        """Log status sekali per LOG_INTERVAL_S (hindari banjir)."""
        now = time.monotonic()
        if now - self._last_log < LOG_INTERVAL_S:
            return
        self._last_log = now
        if self._last_gcs_rx is None:
            rx = "belum ada paket"
        else:
            rx = "%.1fs lalu" % (now - self._last_gcs_rx)
        log.info("mode=%s | QGC(rx %s) | pkt GCS=%d PX=%d",
                 self._mode_txt, rx, self.n_gcs, self.n_px)

    # ---------------------------------------------------------- siklus
    def run(self):
        """Loop utama sampai `stop()` dipanggil."""
        log.info("Starter: tekan Ctrl+C untuk berhenti (kirim netral dulu).")
        while not self._stopping:
            px_ok = self._ensure_px()
            gcs_ok = self._ensure_gcs()
            if px_ok and gcs_ok:
                self._forward(self.gcs, self.px, from_gcs=True)
                self._forward(self.px, self.gcs, from_gcs=False)
                self._failsafe_neutral()
                self._periodic_log()
            time.sleep(POLL_SLEEP_S)

    def stop(self):
        """Set flag berhenti; loop berakhir di akhir putaran aman."""
        self._stopping = True

    def shutdown(self):
        """Kirim netral beberapa kali, lalu tutup kedua koneksi."""
        if self.px is not None:
            for _ in range(3):
                neutral_manual_send(self.px)
                time.sleep(0.05)
        for link in (self.gcs, self.px):
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
        log.info("Selesai (netral sudah dikirim, koneksi ditutup).")


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial", default="",
                   help="port Pixhawk, mis. COM3 (Windows) atau "
                        "/dev/ttyACM0 (Linux). Kosong = auto-detect.")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="baud Pixhawk (USB=115200).")
    p.add_argument("--gcs", default=DEFAULT_GCS,
                   help="link ke QGC, bentuk mavutil. Default %(default)s "
                        "(127.0.0.1 bila QGC di NUC yang sama).")
    p.add_argument("--verbose", action="store_true",
                   help="log debug (forward error, dsb).")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S")
    router = Router(args.serial, args.baud, args.gcs)
    signal.signal(signal.SIGINT, lambda *_: router.stop())
    try:
        signal.signal(signal.SIGTERM, lambda *_: router.stop())
    except (AttributeError, ValueError, OSError):
        pass  # SIGTERM tidak tersedia di semua platform
    try:
        router.run()
    finally:
        router.shutdown()


if __name__ == "__main__":
    main()
