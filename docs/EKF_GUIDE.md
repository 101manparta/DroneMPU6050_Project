# Panduan Extended Kalman Filter (EKF) Drone

## Apa itu EKF?

Extended Kalman Filter (EKF) adalah algoritma estimasi state yang menggabungkan:
- **Prediksi** dari gyroskops (integrasi sudut seiring waktu)
- **Koreksi** dari accelerometer (referensi gravitasi) dan magnetometer (referensi Utara)

EKF mengurangi drift gyro dan noise accelerometer secara optimal.

## State Vector (6 variabel)

| Index | Variabel | Unit | Arti |
|-------|----------|------|------|
| 0 | roll | derajat | Kemiringan samping |
| 1 | pitch | derajat | Kemiringan depan-belakang |
| 2 | yaw | derajat | Putaran searah jarum jam |
| 3 | bias_gx | deg/s | Bias gyro X |
| 4 | bias_gy | deg/s | Bias gyro Y |
| 5 | bias_gz | deg/s | Bias gyro Z |

## Langkah EKF

### 1. Prediction (Prediksi)
Menggunakan ukuran gyro:
```
x_pred = x + dt × (gyro - bias)
P_pred = F × P × Fᵀ + Q
```

### 2. Update Accelerometer
Mengukur arah gravitasi untuk koreksi roll & pitch:
```
z_acc = [ax, ay, az] / |a|
h(x) = [sin(pitch), -sin(roll)·cos(pitch), -cos(roll)·cos(pitch)]
```

### 3. Update Magnetometer
Mengukur medan magnetik Bumi untuk koreksi yaw (heading absolut):
```
z_mag = [mx, my, mz] / |m|
h(x) = Rᵀ × [1, 0, 0]  (model magnetik Utara)
```

Update magnetometer dilakukan setiap 5 sampel untuk mengurangi noise.

## Parameter EKF (sesuai ekf.h)

| Parameter | Nilai | Keterangan |
|-----------|-------|------------|
| Q[0][0], Q[1][1], Q[2][2] | 0.01 | Proses noise attitude |
| Q[3][3], Q[4][4], Q[5][5] | 0.001 | Proses noise bias gyro |
| R_acc[diag] | 0.05 | Measurement noise accel |
| R_mag[diag] | 0.10 | Measurement noise mag |
| P awal (diagonal) | 1.0 | Kovariansi awal (kepercayaan rendah) |

## Tuning EKF

- **Naikkan Q** jika respons terasa terlambat (laggy).
- **Turunkan Q** jika output terasa getar/noisy.
- **Naikkan R_acc** jika accelerometer banyak noise (getaran tinggi).
- **Naikkan R_mag** jika magnetometer terganggu elektronik drone.

## Output EKF

Program C++ host akan menampilkan:
- Estimasi roll, pitch, yaw yang difilter
- Estimasi bias gyro X, Y, Z (deg/s)
- Statistik akurasi (RMSE, Mean Error) vs presisi (Std Dev)

## Perbandingan: Complementary Filter vs EKF

| Aspek | Complementary Filter | EKF |
|-------|---------------------|-----|
| Kompleksitas | Rendah | Tinggi |
| Akurasi | Baik jangka pendek | Baik jangka panjang |
| Bias gyro | Tidak diestimasi | Diestimasi otomatis |
| Magnetometer | Optional | Diintegrasi optimal |
| Komputasi | Sangat ringan | Sedang (matriks 6x6) |