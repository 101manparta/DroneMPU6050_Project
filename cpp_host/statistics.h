#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>

// Hasil analisis statistik akurasi & presisi
struct SensorStats {
    float meanError;     // Akurasi: selisih rata-rata vs nilai referensi
    float rmse;          // Root Mean Square Error
    float stdDev;        // Presisi: deviasi standar (semakin kecil semakin presisi)
    float variance;
    float minError;
    float maxError;
    int sampleCount;
};

// Hitung statistik dari sejumlah error (terukur - referensi)
inline SensorStats computeStats(const std::vector<float>& errors) {
    SensorStats s;
    s.sampleCount = static_cast<int>(errors.size());

    if (s.sampleCount == 0) {
        s.meanError = s.rmse = s.stdDev = s.variance = s.minError = s.maxError = 0.0f;
        return s;
    }

    // Mean error (akurasi)
    float sum = 0.0f;
    for (float e : errors) sum += e;
    s.meanError = sum / s.sampleCount;

    // Variance & Std Dev (presisi)
    float sqDiffSum = 0.0f;
    for (float e : errors) {
        float d = e - s.meanError;
        sqDiffSum += d * d;
    }
    s.variance = sqDiffSum / s.sampleCount;
    s.stdDev = std::sqrtf(s.variance);

    // RMSE
    float sqErrSum = 0.0f;
    for (float e : errors) sqErrSum += e * e;
    s.rmse = std::sqrtf(sqErrSum / s.sampleCount);

    s.minError = *std::min_element(errors.begin(), errors.end());
    s.maxError = *std::max_element(errors.begin(), errors.end());

    return s;
}

// Cetak laporan statistik
inline void printStats(const char* axisName, const SensorStats& s) {
    std::cout << "\n--- " << axisName << " ---\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Samples        : " << s.sampleCount << "\n";
    std::cout << "Mean Error     : " << s.meanError << " deg (Akurasi)\n";
    std::cout << "RMSE           : " << s.rmse << " deg\n";
    std::cout << "Std Dev        : " << s.stdDev << " deg (Presisi)\n";
    std::cout << "Variance       : " << s.variance << "\n";
    std::cout << "Min Error      : " << s.minError << " deg\n";
    std::cout << "Max Error      : " << s.maxError << " deg\n";
}

// Integrasi numerik (metode trapesium)
// values: data sensor, times: waktu detik yang sesuai
inline float trapezoidalIntegral(const std::vector<float>& values,
                                 const std::vector<float>& times) {
    float sum = 0.0f;
    for (size_t i = 1; i < values.size(); i++) {
        float dt = times[i] - times[i - 1];
        sum += 0.5f * (values[i - 1] + values[i]) * dt;
    }
    return sum;
}

// Turunan numerik (pusat, dengan handling batas)
inline float numericalDerivative(const std::vector<float>& values,
                                 const std::vector<float>& times,
                                 size_t idx) {
    if (values.size() < 2) return 0.0f;

    if (idx == 0) {
        return (values[1] - values[0]) / (times[1] - times[0]);
    } else if (idx == values.size() - 1) {
        return (values.back() - values[values.size() - 2]) /
               (times.back() - times[values.size() - 2]);
    } else {
        return (values[idx + 1] - values[idx - 1]) /
               (times[idx + 1] - times[idx - 1]);
    }
}

#endif // STATISTICS_H