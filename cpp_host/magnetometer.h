#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include "matrix.h"

// ============================================================================
// Magnetometer Kalibrasi & Heading (Tilt-Compensated)
// ============================================================================

struct MagCalibration {
    float mxOff, myOff, mzOff;   // hard iron offset
    float mxScale, myScale, mzScale; // soft iron scale
    bool calibrated;

    MagCalibration() : mxOff(0), myOff(0), mzOff(0),
                       mxScale(1), myScale(1), mzScale(1),
                       calibrated(false) {}

    // Apply calibration to raw magnetometer data
    void apply(float& mx, float& my, float& mz) const {
        if (!calibrated) return;
        mx = (mx - mxOff) * mxScale;
        my = (my - myOff) * myScale;
        mz = (mz - mzOff) * mzScale;
    }

    // Hitung parameter kalibrasi dari sejumlah sampel (ellipsoid fitting sederhana)
    void calibrate(const std::vector<float>& mxRaw,
                   const std::vector<float>& myRaw,
                   const std::vector<float>& mzRaw) {
        if (mxRaw.empty()) return;

        float mxMin = *std::min_element(mxRaw.begin(), mxRaw.end());
        float mxMax = *std::max_element(mxRaw.begin(), mxRaw.end());
        float myMin = *std::min_element(myRaw.begin(), myRaw.end());
        float myMax = *std::max_element(myRaw.begin(), myRaw.end());
        float mzMin = *std::min_element(mzRaw.begin(), mzRaw.end());
        float mzMax = *std::max_element(mzRaw.begin(), mzRaw.end());

        mxOff = (mxMax + mxMin) / 2.0f;
        myOff = (myMax + myMin) / 2.0f;
        mzOff = (mzMax + mzMin) / 2.0f;

        float mxRange = mxMax - mxMin;
        float myRange = myMax - myMin;
        float mzRange = mzMax - mzMin;
        float avgRange = (mxRange + myRange + mzRange) / 3.0f;

        mxScale = (mxRange > 0) ? avgRange / mxRange : 1.0f;
        myScale = (myRange > 0) ? avgRange / myRange : 1.0f;
        mzScale = (mzRange > 0) ? avgRange / mzRange : 1.0f;

        calibrated = true;

        std::cout << "\n[Magnetometer Kalibrasi]\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "mxOff=" << mxOff << " myOff=" << myOff << " mzOff=" << mzOff << "\n";
        std::cout << "mxScale=" << mxScale << " myScale=" << myScale << " mzScale=" << mzScale << "\n";
    }
};

// Tilt-compensated heading (derajat)
// roll, pitch dalam derajat; mx, my, mz sudah dikalibrasi
inline float computeHeading(float mx, float my, float mz,
                            float rollDeg, float pitchDeg) {
    float sr = std::sinf(rollDeg * DEG2RAD);
    float cr = std::cosf(rollDeg * DEG2RAD);
    float sp = std::sinf(pitchDeg * DEG2RAD);
    float cp = std::cosf(pitchDeg * DEG2RAD);

    // Tilt compensation
    float Xh = mx * cp + mz * sp;
    float Yh = mx * sr * sp + my * cr - mz * sr * cp;

    float heading = std::atan2f(-Yh, Xh) * RAD2DEG;

    // Normalisasi ke [0, 360)
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    return heading;
}

// Deklinali magnetom (sudut antara medan magnetik Bumi dan bidang horizontal)
// Digunakan untuk koreksi magnetometer jika diperlukan
inline float computeDeclination(float mx, float my, float mz) {
    return std::atan2f(my, mx) * RAD2DEG;
}

#endif // MAGNETOMETER_H