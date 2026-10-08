# thruster_test — Tes thruster MAJU/MUNDUR via Pixhawk (NUC Windows)

Folder uji mandiri untuk **ASUS NUC Windows**: ngetes thruster (maju/mundur)
yang tersambung ke **Pixhawk 6C**. QGC + joystick Xbox jalan di NUC yang sama.

> **PENTING (keselamatan):** alat ini untuk **uji bench darat**.
> **Lepas baling-baling thruster** sebelum menjalankan. Skrip selalu kirim
> netral dulu saat berhenti — tapi itu bukan pengganti kill switch.

---

## Wiring

```
Baterai ──► ESC (kabel power merah/hitam)
Pixhawk MAIN OUT (kabel sinyal putih/kuning) ──► ESC (kabel sinyal)
ESC (3 kabel motor) ──► thruster
Pixhawk USB ──► NUC (kabel USB)
```

- Kabel sinyal ESC colok ke **output channel throttle** Pixhawk. Kalau belum
  tahu channel berapa: buka QGC → Vehicle Setup → Servo Output, gerakkan stik
  throttle, lihat bar mana yang bergerak — colok ESC ke situ.
- Ground wajib sama (satu baterai / BEC yang sama).

---

## Instalasi di NUC (PowerShell)

```powershell
cd servo_qgc
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Cek port Pixhawk

1. Buka **Device Manager → Ports (COM & LPT)** — harus muncul
   `USB Serial Device (COMx)` saat Pixhawk dicolok USB. Catat COM-nya.
2. Atau via PowerShell:
   ```powershell
   [System.IO.Ports.SerialPort]::getportnames()
   ```

> **Aturan emas:** satu port COM hanya bisa dibuka **satu aplikasi**.
> Kalau QGC lagi konek serial ke Pixhawk, Python tidak bisa buka COM yang
> sama (error `PermissionError` / "port dipakai").

---

## Cara pakai

### A) Tes manual pakai Xbox (tanpa Python)

1. QGC konek serial langsung ke Pixhawk (kondisi kamu sekarang — sudah jalan).
2. Set mode **MANUAL**, **arm** via QGC.
3. Gerakkan **stik throttle**: atas = maju, bawah = mundur, tengah = berhenti.
4. Amati arah putaran thruster.

### B) Tes otomatis maju/mundur (pakai Python)

1. Di QGC: **putuskan link SERIAL** ke Pixhawk
   (Application Settings → Comm Links → Disconnect). QGC boleh tetap buka.
2. Jalankan:
   ```powershell
   python sweep.py --serial COM3
   ```
   Ganti `COM3` dengan COM dari Device Manager. Kosongkan `--serial` untuk
   auto-detect.
3. Siklus: `netral → MAJU penuh → netral → MUNDUR penuh → netral`,
   tiap posisi ditahan 2 detik (`--hold 3` untuk 3 detik,
   `--cycles 2` untuk 2 putaran saja).
4. Berhenti dengan **Ctrl+C** — netral otomatis dikirim, thruster berhenti.

### C) Router + failsafe (opsional, lanjutan)

`python main.py --serial COM3` menjembatani QGC ↔ Pixhawk via UDP
`127.0.0.1:14550` (putuskan link serial QGC dulu, QGC pakai link UDP
listen 14550). Kelebihannya: kalau QGC putus > 1,5 detik, skrip otomatis
kirim netral (thruster berhenti).

---

## Troubleshooting

| Gejala | Cek |
|---|---|
| `Port serial tidak ditemukan` | Device Manager → Ports (COM & LPT); cabut-colok USB Pixhawk; coba `--serial COMx` manual |
| `Port dipakai aplikasi lain` | Putuskan link serial QGC dulu (aturan emas di atas) |
| Thruster diam saat MAJU/MUNDUR | Pixhawk belum arm; bukan mode MANUAL; ESC belum kalibrasi / belum bunyi beep; kabel sinyal di channel salah |
| Thruster muter terus setelah ^C | Jangan cabut USB dulu — tunggu log `netral dikirim`; cek ESC kalibrasi |
| QGC tidak dapat telemetri (mode C) | QGC pakai link UDP listen **14550**; IP `--gcs` default `127.0.0.1` sudah benar untuk NUC yang sama; cek firewall Windows untuk UDP |
| `OSError: [WinError 10022]` (versi lama) | Update `main.py` ke versi terbaru (socket UDP sekarang di-bind otomatis + socket error tidak bikin crash) |

### D) Tes thruster via mode OFFBOARD (PX4 v1.17, tanpa GPS)

Dipakai kalau butuh mode Offboard (bukan MANUAL). Prinsip PX4: **stream
setpoint dulu, baru boleh pindah mode** — kalau belum ada stream, QGC
menolak dengan `Switching to mode 'Offboard' is currently not possible:
No offboard signal`.

1. Set sekali via QGC → Parameters (lalu **reboot** Pixhawk):
   - `MAV_FWDEXTSP = 1` — teruskan setpoint eksternal ke modul rover
     (default 0 = setpoint dibuang, Offboard mustahil).
   - `COM_ARM_WO_GPS = 1` — boleh arm tanpa GPS (bench indoor).
2. Putuskan link serial QGC, lalu:
   ```powershell
   python offboard_thrust.py --serial COM3
   ```
3. Alur otomatis: stream vx=0 @10Hz selama 3 detik → masuk OFFBOARD →
   hitung mundur 3 detik → arm. Kontrol: `W/S` tambah/kurang maju
   (default langkah 0.5 m/s, maks 2 m/s), `X`/Spasi berhenti,
   `O` masuk Offboard lagi kalau kepental, `Q` keluar aman
   (vx=0 → MANUAL → disarm).
4. Verifikasi tanpa gerak dulu (aman):
   ```powershell
   python offboard_thrust.py --serial COM3 --no-arm
   ```
   QGC harus tampil mode **Offboard**. Kalau tetap ditolak, baca log
   script: GPS/EKF ikut dilaporkan tiap start.
5. Catatan tanpa GPS: Offboard butuh estimasi gerak minimal yang valid
   (attitude + heading). Kalau EKF tetap menolak, fallback-nya mode
   MANUAL (cara A) — itu selalu bisa.

| Gejala (Offboard) | Cek |
|---|---|
| `No offboard signal` | Script **belum jalan** saat pencet mode di QGC — stream harus dari `offboard_thrust.py`, bukan dari QGC/Xbox; `MAV_FWDEXTSP` harus 1 + reboot |
| `Gagal masuk OFFBOARD` | `MAV_FWDEXTSP=1` + reboot; cek log GPS/EKF di awal script |
| `ARM ditolak` | `COM_ARM_WO_GPS=1` + reboot; cek pre-arm check di QGC |
| Kepental keluar Offboard | Stream putus > `COM_OF_LOSS_T` — jangan tutup/suspend script; tekan `O` untuk masuk lagi |

---

## Uji (developer)

```powershell
python -m unittest discover -s tests -v
```
