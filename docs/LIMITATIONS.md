# Batasan Project (Penting Dibaca)

## Akurasi 100% Tidak Mungkin dengan MPU6050

MPU6050 adalah sensor **MEMS (Micro-Electro-Mechanical Systems)** kelas konsumen. Ia bukan sensor navigasi kelas industri atau militer. Oleh karena itu:

1. **Gyro Drift**: Setiap integrasi waktu pada gyro akan menumpuk error. Seiring waktu, sudut yaw akan melesat meskipun drone diam.
2. **No Magnetometer**: Tanpa kompas digital, yaw tidak memiliki referensi absolut ke Utara.
3. **Sensitif Suhu**: Output sensor berubah seiring suhu berubah.
4. **Getaran**: Motor drone menyebabkan getaran yang memicu noise accelerometer.

## Akurasi vs Presisi

- **Akurasi**: seberapa dekat hasil pengukuran dengan nilai sebenarnya (ground truth).
- **Presisi**: seberapa konsisten hasil pengukuran yang diulang-ulang.

MPU6050 biasanya memiliki presisi yang baik pada jangka pendek, tetapi akurasi menurun seiring waktu karena drift.

## Kecepatan Cahaya

Konstanta `c = 299.792.458 m/s` digunakan dalam project ini untuk simulasi **Time-of-Flight (ToF)**:

```
jarak = (c × waktu) / 2
```

Ini hanya **simulasi matematis**. Untuk ToF nyata pada drone, diperlukan sensor jarak khusus seperti:
- VL6180X
- TF-Luna (LiDAR)
- Benewake, Garmin LiDAR, dll.

## Rekomendasi untuk Akurasi Tinggi

Jika Anda membutuhkan akurasi tinggi untuk drone nyata:

1. Tambahkan **magnetometer (HMC5883L / QMC5883L)** untuk referensi yaw.
2. Tambahkan **barometer (BMP280 / MS5611)** untuk estimasi ketinggian.
3. Gunakan **Extended Kalman Filter (EKF)** bukan hanya complementary filter.
4. Gunakan sensor IMU kelas tinggi (misalnya ICM-20948, BMI160, atau navigate-grade IMU).
5. Lakukan **sensor fusion** dengan GPS untuk navigasi eksternal.

## Kesimpulan

Project ini cocok untuk:
- Belajar kalkulus, vektor, dan matriks dalam konteks drone.
- Memahami akurasi vs presisi data sensor.
- Prototipe dan uji coba konsep.

Project ini **tidak cocok** untuk aplikasi penerbangan autonom nyata tanpa penambahan sensor dan filter yang lebih canggih.