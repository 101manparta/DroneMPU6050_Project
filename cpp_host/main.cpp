/*
 * DroneMPU6050 C++ Host Analyzer — Full Version
 * Program untuk mengolah data serial dari Arduino MPU6050 + HMC5883L.
 * 
 * Fitur:
 * - Membaca file CSV hasil capture dari Arduino
 * - Extended Kalman Filter (EKF) untuk attitude estimation (roll, pitch, yaw + bias)
 * - Magnetometer untuk referensi yaw absolut (tilt-compensated heading)
 * - Modeling attitudo dengan matriks rotasi 3x3
 * - Kalkulus: integrasi & turunan numerik
 * - Analisis akurasi vs presisi (RMSE, Mean Error, Std Dev)
 * - Simulasi Time-of-Flight menggunakan kecepatan cahaya (c)
 * - Generate file HTML untuk simulasi drone 3D
 * - Output laporan ke console dan file laporan_analisis.txt
 *
 * Kompilasi (Windows, g++):
 *   g++ -std=c++11 -O2 main.cpp -o drone_analyzer.exe
 *
 * Kompilasi (Linux/macOS):
 *   g++ -std=c++11 -O2 main.cpp -o drone_analyzer
 *
 * Usage:
 *   drone_analyzer.exe drone_data.csv [output_3d.html]
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <ctime>

#include "matrix.h"
#include "statistics.h"
#include "ekf.h"
#include "magnetometer.h"

// Konstanta fisika
const double LIGHT_SPEED = 299792458.0; // m/s, kecepatan cahaya dalam vakum

struct DroneSample {
    double timestamp_ms;
    float roll, pitch, yaw;       // data mentah dari sensor (complementary filter)
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float temp_c;
    float heading_mag;            // heading dari magnetometer (tilt-compensated)
};

// Parsi 1 baris CSV dari Arduino (format EKF sketch)
bool parseCSVLine(const std::string& line, DroneSample& s) {
    std::stringstream ss(line);
    std::string token;
    auto getNext = [&]() -> bool {
        if (!std::getline(ss, token, ',')) return false;
        return true;
    };
    if (!getNext()) return false; s.timestamp_ms = std::stod(token);
    if (!getNext()) return false; s.roll = std::stof(token);
    if (!getNext()) return false; s.pitch = std::stof(token);
    if (!getNext()) return false; s.yaw = std::stof(token);
    if (!getNext()) return false; s.ax = std::stof(token);
    if (!getNext()) return false; s.ay = std::stof(token);
    if (!getNext()) return false; s.az = std::stof(token);
    if (!getNext()) return false; s.gx = std::stof(token);
    if (!getNext()) return false; s.gy = std::stof(token);
    if (!getNext()) return false; s.gz = std::stof(token);
    if (!getNext()) return false; s.mx = std::stof(token);
    if (!getNext()) return false; s.my = std::stof(token);
    if (!getNext()) return false; s.mz = std::stof(token);
    if (!getNext()) return false; s.temp_c = std::stof(token);
    if (!getNext()) return false; s.heading_mag = std::stof(token);
    return true;
}

// Baca file CSV
std::vector<DroneSample> loadCSV(const std::string& filename) {
    std::vector<DroneSample> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: tidak bisa membuka file " << filename << "\n";
        return data;
    }

    std::string line;
    bool headerSkipped = false;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (!headerSkipped) {
            std::stringstream check(line);
            std::string first;
            std::getline(check, first, ',');
            try {
                std::stod(first);
                headerSkipped = true;
            } catch (...) {
                headerSkipped = true;
                continue;
            }
        }

        DroneSample s;
        if (parseCSVLine(line, s)) {
            data.push_back(s);
        }
    }

    return data;
}

// Simulasi jarak dengan Time-of-Flight (kecepatan cahaya)
float calculateToFDistance(float timeNanoseconds) {
    float seconds = timeNanoseconds * 1e-9f;
    return static_cast<float>((LIGHT_SPEED * seconds) / 2.0);
}

// Generate file HTML untuk visualisasi 3D (panggil drone_3d_viz.logik secara inline)
// Untuk kesederhanaan, kita generate data JS yang bisa di-copy ke drone_3d.html
void generate3DDataJS(const std::vector<DroneSample>& samples, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Gagal membuat file " << filename << "\n";
        return;
    }

    file << "// Data drone untuk visualisasi 3D\n";
    file << "const droneData3D = [\n";
    for (size_t i = 0; i < samples.size(); i++) {
        const auto& s = samples[i];
        file << "  { t: " << s.timestamp_ms
             << ", roll: " << s.roll
             << ", pitch: " << s.pitch
             << ", yaw: " << s.yaw
             << ", heading: " << s.heading_mag
             << ", ax: " << s.ax << ", ay: " << s.ay << ", az: " << s.az
             << " }";
        if (i + 1 < samples.size()) file << ",";
        file << "\n";
    }
    file << "];\n";
    file.close();
    std::cout << "Data 3D disimpan ke: " << filename << "\n";
}

int main(int argc, char* argv[]) {
    std::string inputFile = "drone_data.csv";
    std::string output3D = "drone_3d_data.js";

    if (argc > 1) inputFile = argv[1];
    if (argc > 2) output3D = argv[2];

    std::cout << "============================================================\n";
    std::cout << "   DRONE MPU6050 + MAG — EKF, AKURASI, PRESI, DAN 3D VIZ   \n";
    std::cout << "============================================================\n\n";

    // 1. Muat data
    std::vector<DroneSample> samples = loadCSV(inputFile);
    if (samples.empty()) {
        std::cerr << "Tidak ada data yang dimuat.\n";
        std::cerr << "Contoh: drone_analyzer.exe drone_data.csv output_3d.js\n";
        return 1;
    }

    std::cout << "Data dimuat: " << samples.size() << " sampel dari " << inputFile << "\n";
    std::cout << "Waktu mulai : " << samples.front().timestamp_ms << " ms\n";
    std::cout << "Waktu selesai: " << samples.back().timestamp_ms << " ms\n\n";

    // 2. Kalibrasi Magnetometer (dari data mentah)
    MagCalibration magCal;
    std::vector<float> mxRaw, myRaw, mzRaw;
    for (const auto& s : samples) {
        mxRaw.push_back(s.mx);
        myRaw.push_back(s.my);
        mzRaw.push_back(s.mz);
    }
    magCal.calibrate(mxRaw, myRaw, mzRaw);

    // 3. Inisialisasi EKF
    EKF ekf;
    ekf.setDT(0.01f); // 100 Hz

    // Inisialisasi EKF dari sampel pertama
    float mxCal = samples[0].mx, myCal = samples[0].my, mzCal = samples[0].mz;
    magCal.apply(mxCal, myCal, mzCal);
    ekf.initialize(samples[0].ax, samples[0].ay, samples[0].az,
                   mxCal, myCal, mzCal);

    // 4. Jalankan EKF pada seluruh data
    std::vector<float> ekfRoll, ekfPitch, ekfYaw;
    std::vector<float> ekfBiasX, ekfBiasY, ekfBiasZ;
    std::vector<float> timesSec;

    // Variabel untuk analisis akurasi vs referensi (nilai ground truth dapat diubah)
    const float TRUE_ROLL  = 0.0f;
    const float TRUE_PITCH = 0.0f;
    const float TRUE_YAW   = 0.0f;

    std::vector<float> rollErr, pitchErr, yawErr;

    for (size_t i = 0; i < samples.size(); i++) {
        const auto& s = samples[i];
        timesSec.push_back(static_cast<float>(s.timestamp_ms) / 1000.0f);

        // Kalibrasi magnetometer per sampel
        float mx = s.mx, my = s.my, mz = s.mz;
        magCal.apply(mx, my, mz);

        // EKF Predict (dari gyro)
        ekf.predict(s.gx, s.gy, s.gz);

        // EKF Update (dari accelerometer — setiap sampel)
        ekf.updateAccel(s.ax, s.ay, s.az);

        // EKF Update (dari magnetometer — setiap 5 sampel untuk mengurangi noise)
        if (i % 5 == 0) {
            ekf.updateMag(mx, my, mz);
        }

        // Simpan hasil
        ekfRoll.push_back(ekf.getRoll());
        ekfPitch.push_back(ekf.getPitch());
        ekfYaw.push_back(ekf.getYaw());
        ekfBiasX.push_back(ekf.getBiasX());
        ekfBiasY.push_back(ekf.getBiasY());
        ekfBiasZ.push_back(ekf.getBiasZ());

        // Error vs referensi
        rollErr.push_back(ekf.getRoll() - TRUE_ROLL);
        pitchErr.push_back(ekf.getPitch() - TRUE_PITCH);
        yawErr.push_back(ekf.getYaw() - TRUE_YAW);
    }

    // 5. Statistik akurasi & presisi (setelah EKF)
    SensorStats statRoll = computeStats(rollErr);
    SensorStats statPitch = computeStats(pitchErr);
    SensorStats statYaw = computeStats(yawErr);

    std::cout << "\n============================================================\n";
    std::cout << "              EKF — HASIL ESTIMASI ATTITUDE                 \n";
    std::cout << "============================================================\n";
    printStats("ROLL (EKF)", statRoll);
    printStats("PITCH (EKF)", statPitch);
    printStats("YAW (EKF)", statYaw);

    // Bias gyro estimasi
    std::cout << "\n--- Estimasi Bias Gyro (EKF) ---\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Bias X: " << ekfBiasX.back() << " deg/s\n";
    std::cout << "Bias Y: " << ekfBiasY.back() << " deg/s\n";
    std::cout << "Bias Z: " << ekfBiasZ.back() << " deg/s\n";

    // 6. Modeling matriks rotasi (dari hasil EKF)
    std::cout << "\n============================================================\n";
    std::cout << "              MODELING MATRIKS ROTASI 3X3                   \n";
    std::cout << "============================================================\n";
    Mat3 R = attitudeMatrix(ekf.getRoll(), ekf.getPitch(), ekf.getYaw());
    R.print("Matriks Rotasi Akhir (R = Rz*Ry*Rx)");

    // Transformasi vektor gravitasi world [0,0,1] ke body frame
    Vec3 gravityWorld(0, 0, 1);
    Vec3 gravityBody = R * gravityWorld;
    gravityBody.print("Gravitasi di Body Frame");

    // 7. Kalkulus numerik
    std::cout << "\n============================================================\n";
    std::cout << "                    KALKULUS NUMERIK                          \n";
    std::cout << "============================================================\n";
    std::vector<float> gzValues;
    for (const auto& s : samples) gzValues.push_back(s.gz);
    float yawIntegrated = trapezoidalIntegral(gzValues, timesSec);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Integrasi Gyro Z (total yaw estimasi): " << yawIntegrated << " derajat\n";

    std::vector<float> azValues;
    for (const auto& s : samples) azValues.push_back(s.az);
    size_t mid = azValues.size() / 2;
    float azDerivative = numericalDerivative(azValues, timesSec, mid);
    std::cout << "Turunan Accel Z pada sampel tengah (dz/dt): " << azDerivative << " g/s\n";

    // 8. Kecepatan cahaya (ToF simulasi)
    std::cout << "\n============================================================\n";
    std::cout << "              KECEPATAN CAHAYA (Time-of-Flight)             \n";
    std::cout << "============================================================\n";
    std::cout << "Kecepatan cahaya c = " << LIGHT_SPEED << " m/s\n";
    float tofNs = 100.0f;
    float distance = calculateToFDistance(tofNs);
    std::cout << "Waktu perpindahan (pulang pergi): " << tofNs << " ns\n";
    std::cout << "Jarak estimasi drone-ke-target : " << distance << " meter\n";
    std::cout << "Catatan: simulasi matematis. Untuk ToF nyata perlukan sensor VL6180X/TF-Luna/LiDAR.\n";

    // 9. Generate data untuk visualisasi 3D
    std::cout << "\n============================================================\n";
    std::cout << "              GENERATE DATA VISUALISASI 3D                  \n";
    std::cout << "============================================================\n";
    generate3DDataJS(samples, output3D);

    // 10. Simpan laporan ke file
    std::ofstream report("laporan_analisis.txt");
    if (report.is_open()) {
        report << "LAPORAN ANALISIS DRONE MPU6050 + HMC5883L\n";
        report << "Tanggal: " << __DATE__ << " " << __TIME__ << "\n\n";
        report << "Total sampel: " << samples.size() << "\n\n";

        report << "--- KALIBRASI MAGNETOMETER ---\n";
        report << std::fixed << std::setprecision(4);
        report << "mxOff=" << magCal.mxOff << " myOff=" << magCal.myOff << " mzOff=" << magCal.mzOff << "\n";
        report << "mxScale=" << magCal.mxScale << " myScale=" << magCal.myScale << " mzScale=" << magCal.mzScale << "\n\n";

        report << "--- EKF HASIL ESTIMASI ---\n";
        report << "ROLL  -> Mean Error: " << statRoll.meanError << ", RMSE: " << statRoll.rmse
               << ", StdDev: " << statRoll.stdDev << "\n";
        report << "PITCH -> Mean Error: " << statPitch.meanError << ", RMSE: " << statPitch.rmse
               << ", StdDev: " << statPitch.stdDev << "\n";
        report << "YAW   -> Mean Error: " << statYaw.meanError << ", RMSE: " << statYaw.rmse
               << ", StdDev: " << statYaw.stdDev << "\n\n";

        report << "--- BIAS GYRO (EKF) ---\n";
        report << "Bias X: " << ekfBiasX.back() << " deg/s\n";
        report << "Bias Y: " << ekfBiasY.back() << " deg/s\n";
        report << "Bias Z: " << ekfBiasZ.back() << " deg/s\n\n";

        report << "--- KALKULUS ---\n";
        report << "Integrasi Gyro Z: " << yawIntegrated << " derajat\n";
        report << "Turunan Accel Z (tengah): " << azDerivative << " g/s\n\n";

        report << "--- KECEPATAN CAHAYA (ToF 100ns) ---\n";
        report << "Jarak: " << distance << " meter\n";
        report.close();
        std::cout << "\nLaporan disimpan ke: laporan_analisis.txt\n";
    }

    std::cout << "\n============================================================\n";
    std::cout << "SELESAI. File output:\n";
    std::cout << "  - laporan_analisis.txt\n";
    std::cout << "  - " << output3D << " (data untuk drone_3d.html)\n";
    std::cout << "============================================================\n";

    return 0;
}