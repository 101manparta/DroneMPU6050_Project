# Panduan Kalibrasi MPU6050 + HMC5883L

Sebelum drone digunakan, semua sensor harus dikalibrasi untuk menghilangkan offset pabrik dan bias awal.

## 1. Kalibrasi MPU6050 (Otomatis)

Sketch `MPU6050_Drone_EKF.ino` sudah memiliki fungsi `calibrateMPU()` yang dijalankan otomatis saat nyala.

1. Letakkan drone pada **permukaan rata dan diam**.
2. Nyalakan Arduino.
3. Tunggu hingga Serial Monitor menampilkan: `Kalibrasi MPU6050 selesai.`
4. Jangan gerakkan sensor selama kalibrasi berlangsung (sekitar 2 detik).

Parameter yang dikalibrasi:
| Parameter | Arti |
|-----------|------|
| `axOff`, `ayOff`, `azOff` | Offset accelerometer X, Y, Z (dikurangi 1g pada Z) |
| `gxOff`, `gyOff`, `gzOff` | Offset gyroskops X, Y, Z (bias nol) |

## 2. Kalibrasi Magnetometer HMC5883L (Manual)

Magnetometer perlu kalibrasi **hard iron** (offset) dan **soft iron** (skala) karena distorsi medan magnetik dari komponen elektronik drone.

### Langkah:

1. Buka Serial Monitor Arduino.
2. Lihat output kalibrasi magnetometer (sketch akan meminta putar drone 360 derajat).
3. Putar drone dalam **semua arah** (pitch, roll, yaw) selama 30 detik sehingga ellipsoid data tercapai.
4. Tunggu hingga muncul nilai kalibrasi:
   ```
   mxOff=... myOff=... mzOff=...
   mxScale=... myScale=... mzScale=...
   ```
5. Catat nilai-nilai ini untuk digunakan di program C++ host.

### Tips kalibrator magnetometer:

- Putar drone seperti membuka botol (garis sumbu semua tercover).
- Hindari dekat dari logam besar, speaker, atau magnet saat kalibrasi.
- Kalibrasi ulang jika drone dipindahkan lokasi geografis yang jauh (declinasi magnetik berubah).

## 3. EKF (Extended Kalman Filter)

Program C++ host menjalankan EKF pada data yang sudah dicapture. Tidak perlu kalibrasi tambahan untuk EKF — ia menggunakan:
- Output MPU6050 yang sudah dikalibrasi di Arduino
- Output magnetometer yang sudah dikalibrasi

State EKF: `[roll, pitch, yaw, bias_gx, bias_gy, bias_gz]`

EKF akan estimasi bias gyro secara otomatis selama berjalan.

## 4. Verifikasi Kalibrasi

Setelah kalibrasi, letakkan drone diam pada lantai datar:
- Roll dan pitch harus mendekati 0°
- Yaw/Heading harus stabil dan sesuai kompas fisik
- Jika drift tinggi, ulangi kalibrasi MPU6050