# servo_qgc — Bench "gerakkan servo lewat QGC + joystick"

Folder uji mandiri untuk **mini PC / Raspberry Pi**: menjembatani laptop
QGC (yang joystick-nya sudah dikalibrasi) dengan Pixhawk lewat MAVLink,
sehingga **servo bisa digerakkan dari joystick di QGC**. Semua kode di sini
berdiri sendiri (tidak depend ke repo utama) — tinggal salin folder ini ke
mini PC.

> **PENTING (keselamatan):** alat ini untuk **uji bench darat**. Servo /
> ESC tetap digerakkan **oleh Pixhawk** (PWM dilakukan Pixhawk); skrip
> hanya meneruskan dan memantau sinyal MAVLink — bukan pengganti
> E-stop / switch RC / kill switch bawaan firmware.

---

## Cara kerja

```
laptop: QGC (Applications > Comm Links > UDP, listen 14550)
        ├─ joystick Xbox dikalibrasi di QGC Settings > Joystick
        │
        │  UDP 14550  (QGC mengirim MANUAL_CONTROL + terima telemetri)
        ▼
mini PC:  main.py  = router MAVLink dua arah
        │            • QGC → Pixhawk : semua paket diteruskan (MANUAL_CONTROL dipantau → v,w)
        │            • Pixhawk → QGC : telemetri diteruskan (HEARTBEAT → tampilkan mode)
        │            • FAILSAFE      : QGC putus > 1,5 dtk → kirim netral periodik
        ▼
      Pixhawk (mode MANUAL)  ── PWM ──►  servo arah / ESC thrust
```

Skala sinyal (sesuai protokol MAVLink `MANUAL_CONTROL`):
- `z` throttle (0–1000, netral 500) → **v = surge** (maju/mundur)
- `r` yaw (±127) → **w = yaw** (belok kiri/kanan)

---

## Isi folder

| File | Fungsi |
|---|---|
| `main.py` | Router MAVLink QGC↔Pixhawk + monitor (v,w) + failsafe netral |
| `sweep.py` | Tes gerak servo kiri-kanan otomatis (tanpa QGC) |
| `demo_loop.py` | Latihan stik via keyboard, tanpa hardware/QGC |
| `mavlink_util.py` | Konversi & utilitas MAVLink (dipakai semua skrip) |
| `tests/` | Unit test konversi |
| `requirements.txt` | Dependensi (pymavlink, pyserial) |

---

## Persyaratan

1. **Mini PC / RPi** dengan Linux + Python 3.8+.
2. **Pixhawk** terhubung ke mini PC (USB) atau melalui telemetri radio
   (UART). Usb default 115200 baud; TELEM2 sering 57600.
3. **Servo arah** di output Pixhawk yang dipetakan ke channel **steering
   / yaw** (cek frame: Manual | Plane & Rover → SERVO output). Thrust
   (ESC) sebaiknya belum dipasang saat bench pertama.
4. **Laptop** ber-QGC + joystick (Xbox) yang **sudah dikalibrasi**
   (QGC Settings → Joystick: pilih joystick → Calibrate → semua sumbu).

### Wiring singkat

- Servo (`GND/VCC/Signal`) → port output Pixhawk channel steering.
  **Jangan** dicolok ke rail berkekuatan besar tanpa penerangan
  (pakai BEC/UBEC dengan ground sama yang aman).
- Pixhawk USB (atau TELEM2) → mini PC.
- Mini PC & laptop satu jaringan (Wi-Fi router).
- **Lepas baling-baling / jauhkan benda** sebelum dinyalakan.

---

## Instalasi di mini PC

```bash
cd servo_qgc
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

> `python3-venv` kalau belum ada: `sudo apt install python3-venv`.

---

## Menjalankan

### 0) Cek dulu tanpa hardware (disarankan)

```bash
.venv/bin/python3 demo_loop.py
```
Tekan `W/S/A/D`, `SPASI` (netral), `Q` (keluar). Ini melatih pemetaan
stik → (v,w) dengan skala identik yang nanti dikirim ke Pixhawk.

### 1) Hidupkan Pixhawk + servo

Nyalakan Pixhawk (pastikan switch RC/kill dalam kondisi aman), servo ikut
stabil. Lihat di QGC laptop bahwa Pixhawk muncul & **armed**.

### 2) Jalankan router di mini PC

```bash
# QGC ada di mini PC yang sama:
.venv/bin/python3 main.py

# QGC di laptop lain (ganti IP laptop):
.venv/bin/python3 main.py --gcs udpout:192.168.1.50:14550

# Port serial tidak ke-detect? Beri tahu path-nya:
.venv/bin/python3 main.py --serial /dev/ttyACM0 --baud 115200
```

Log per detik menampilkan: `mode = [...]`, jumlah paket, dan `v / w`
terakhir. Contoh keluaran sehat:

```
[INFO] Link GCS aktif: udpout:127.0.0.1:14550 ...
[INFO] Pixhawk terhubung via /dev/ttyACM0 @ 115200 baud
[INFO] mode= [MANUAL] custom=0 ARMED | QGC(rx 0.2s lalu) | pkt GCS=420 PX=910 MAN=112 | v=+0.00 w=+0.12
```

### 3) Hubungkan QGC laptop

QGC → `Applications Settings → Comm Links → Add → UDP` → port **14550**.
Nyalakan link itu. Telemetri Pixhawk akan muncul di QGC (mode, attitude,
dll).

### 4) Kendali manual

1. Set mode Pixhawk ke **MANUAL/GUIDED manual** lewat QGC (Flight mode).
2. Gerakkan joystick: **throttle stik kiri** = maju/mundur, **stik kanan**
   = kiri/kanan.
3. Servo arah ikut bergerak; di terminal mini PC lihat `v / w` berubah.

### 5) Failsafe (uji wajib)

Matikan link QGC (Disconnect) — dalam **≤ 1,5 detik** terminal mencetak
peringatan `QGC tidak mengirim... -> netral` dan skrip terus mengirim
netral sampai QGC hidup lagi. Servo diharapkan kembali ke posisi tengah.

---

## Tes servo otomatis (menggerakkan kiri-kanan)

```bash
.venv/bin/python3 sweep.py                 # auto-detect port
.venv/bin/python3 sweep.py --baud 57600    # kalau via TELEM2
```

Siklus: netral → kanan penuh → netral → kiri penuh → netral (tiap posisi
ditahan 2 detik). Gunakan untuk memastikan **arah & batas servo** benar.

---

## Pemetaan tongkat QGC → gerak

| QGC (default joystick) | Pesan MAVLink | Gerak kapal |
|---|---|---|
| Stik kiri vertikal (throttle) | `z` 0–1000 | **v** = surge (maju/mundur) |
| Stik kanan horizontal (yaw) | `r` ±127 | **w** = yaw (belok kiri/kanan) |
| Stik lain / tombol | `x/y/buttons` | diabaikan (ASV 2 DOF) |

Jika arah terbalik (stik kanan = belok kiri), periksa **arah channel
servo** di QGC (REVERSED) atau balik arah `r` di lapangan lewat opsi
`--reverse` pada versi berikutnya — lebih baik balik di QGC.

---

## Troubleshooting

| Gejala | Cek |
|---|---|
| `Belum ada port serial terdeteksi` | `ls /dev/serial/by-id/`, `ls /dev/ttyACM*`; kalau tetap tidak ada: `--serial /dev/ttyACM0` |
| QGC tidak dapat telemetri | Link UDP listen 14550 di QGC; IP `--gcs` benar; firewall tidak memblokir |
| Servo diam, w berubah di log | Pixhawk belum armed; mode bukan MANUAL; channel servo salah; servo perlu BEC |
| Log `Forward gagal` beruntun | Link QGC belum aktif — nyalakan Comm Link dulu (aman, otomatis retry) |
| `v=+/-` penuh walau stik di tengah | Kalibrasi ulang joystick di QGC (center/deadband) |
| Peringatan `QGC tidak mengirim` terus | Cek kabel/network; netral terus dikirim (itulah failsafe-nya) |

---

## Catatan teknis

- Router meneruskan paket **bit-for-bit** (`get_msgbuf()`) sehingga QGC
  tetap melihat koneksi penuh (MAVLink1/2 otomatis).
- Failsafe hanya aktif setelah QGC pernah mengirim (agar tidak menabrak
  setup sebelum QGC nyambung).
- `Ctrl+C` pada `main.py` & `sweep.py` selalu mengirim netral dulu lalu
  menutup koneksi.

## Uji

```bash
cd servo_qgc
python3 -m unittest discover -s tests -v
```