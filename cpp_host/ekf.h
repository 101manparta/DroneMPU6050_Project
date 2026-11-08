#ifndef EKF_H
#define EKF_H

#include "matrix.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

// ============================================================================
// Extended Kalman Filter untuk Attitude Estimation Drone (9-DOF)
// State: [roll, pitch, yaw, bias_gx, bias_gy, bias_gz]
// ============================================================================

class EKF {
public:
    // State vector
    float roll, pitch, yaw;           // derajat
    float bgx, bgy, bgz;              // gyro bias (deg/s)

    // Covariance matrix 6x6
    float P[6][6];

    // Process noise covariance Q
    float Q[6][6];

    // Measurement noise R
    float R_acc[3][3];
    float R_mag[3][3];

    bool initialized;
    float dt;

    EKF() : initialized(false), dt(0.01f) {
        roll = pitch = yaw = 0.0f;
        bgx = bgy = bgz = 0.0f;

        // P awal
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P[i][j] = (i == j) ? 1.0f : 0.0f;

        // Q - proses noise
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) Q[i][j] = 0.0f;
        Q[0][0] = 0.01f; Q[1][1] = 0.01f; Q[2][2] = 0.01f;
        Q[3][3] = 0.001f; Q[4][4] = 0.001f; Q[5][5] = 0.001f;

        // R - measurement noise
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                R_acc[i][j] = (i == j) ? 0.05f : 0.0f;
                R_mag[i][j] = (i == j) ? 0.10f : 0.0f;
            }
    }

    void setDT(float dt_) { dt = dt_; }

    // ========================================================================
    // PREDICTION STEP (gunakan gyro)
    // ========================================================================
    void predict(float gx, float gy, float gz) {
        float wx = gx - bgx;
        float wy = gy - bgy;
        float wz = gz - bgz;

        roll  += wx * dt;
        pitch += wy * dt;
        yaw   += wz * dt;

        wrapAngle(roll);
        wrapAngle(pitch);
        wrapAngle(yaw);

        // Jacobian F (6x6)
        float F[6][6] = {
            {1, 0, 0, -dt, 0, 0},
            {0, 1, 0, 0, -dt, 0},
            {0, 0, 1, 0, 0, -dt},
            {0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 1}
        };

        // P = F * P * F^T + Q
        float P_new[6][6] = {{0}};
        float FP[6][6] = {{0}};

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    FP[i][j] += F[i][k] * P[k][j];

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    P_new[i][j] += FP[i][k] * F[j][k];

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P[i][j] = P_new[i][j] + Q[i][j];
    }

    // ========================================================================
    // UPDATE STEP dengan Accelerometer
    // ========================================================================
    void updateAccel(float ax, float ay, float az) {
        float norm = std::sqrtf(ax * ax + ay * ay + az * az);
        if (norm == 0) return;
        ax /= norm; ay /= norm; az /= norm;

        float sr = std::sinf(roll * DEG2RAD);
        float cr = std::cosf(roll * DEG2RAD);
        float sp = std::sinf(pitch * DEG2RAD);
        float cp = std::cosf(pitch * DEG2RAD);

        float hx = sp;
        float hy = -sr * cp;
        float hz = -cr * cp;

        float z[3] = {ax, ay, az};
        float y[3] = {z[0] - hx, z[1] - hy, z[2] - hz};

        // Jacobian H (3x6)
        float H[3][6] = {
            {0, cp, 0, 0, 0, 0},
            {-cr * cp, sr * sp, 0, 0, 0, 0},
            {sr * cp, cr * sp, 0, 0, 0, 0}
        };

        // S = H * P * H^T + R
        float HP[3][6] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    HP[i][j] += H[i][k] * P[k][j];

        float S[3][3] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 6; k++)
                    S[i][j] += HP[i][k] * H[j][k];

        for (int i = 0; i < 3; i++) S[i][i] += R_acc[i][i];

        // PHt = P * H^T
        float PHt[6][3] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 6; k++)
                    PHt[i][j] += P[i][k] * H[j][k];

        // S_inv
        float S_inv[3][3];
        if (!invertMatrix3x3(S, S_inv)) return;

        // K = PHt * S_inv (6x3)
        float K[6][3] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    K[i][j] += PHt[i][k] * S_inv[k][j];

        // Update state
        float correction[6] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                correction[i] += K[i][j] * y[j];

        roll += correction[0];
        pitch += correction[1];
        yaw += correction[2];
        bgx += correction[3];
        bgy += correction[4];
        bgz += correction[5];

        wrapAngle(roll);
        wrapAngle(pitch);
        wrapAngle(yaw);

        // P = (I - K*H) * P
        float KH[6][6] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 3; k++)
                    KH[i][j] += K[i][k] * H[k][j];

        float P_new[6][6] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                float val = 0;
                for (int k = 0; k < 6; k++)
                    val += ((i == k ? 1.0f : 0.0f) - KH[i][k]) * P[k][j];
                P_new[i][j] = val;
            }

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P[i][j] = P_new[i][j];
    }

    // ========================================================================
    // UPDATE STEP dengan Magnetometer
    // ========================================================================
    void updateMag(float mx, float my, float mz) {
        float norm = std::sqrtf(mx * mx + my * my + mz * mz);
        if (norm == 0) return;
        mx /= norm; my /= norm; mz /= norm;

        float sr = std::sinf(roll * DEG2RAD);
        float cr = std::cosf(roll * DEG2RAD);
        float sp = std::sinf(pitch * DEG2RAD);
        float cp = std::cosf(pitch * DEG2RAD);
        float sy = std::sinf(yaw * DEG2RAD);
        float cy = std::cosf(yaw * DEG2RAD);

        // Prediksi magnetometer di body frame (m_world = [1,0,0])
        float hx = cy * cp;
        float hy = cy * sp * sr - sy * cr;
        float hz = cy * sp * cr + sy * sr;

        float z[3] = {mx, my, mz};
        float y[3] = {z[0] - hx, z[1] - hy, z[2] - hz};

        // Jacobian H (3x6)
        float H[3][6] = {
            {0, -cy * sp, -sy * cp, 0, 0, 0},
            {-cy * sp * cr + sy * sr, cy * cp * sr, cy * sp * cr - sy * sr, 0, 0, 0},
            {-cy * sp * sr - sy * cr, cy * cp * cr, -cy * sp * sr + sy * cr, 0, 0, 0}
        };

        float HP[3][6] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    HP[i][j] += H[i][k] * P[k][j];

        float S[3][3] = {{0}};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 6; k++)
                    S[i][j] += HP[i][k] * H[j][k];

        for (int i = 0; i < 3; i++) S[i][i] += R_mag[i][i];

        float PHt[6][3] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 6; k++)
                    PHt[i][j] += P[i][k] * H[j][k];

        float S_inv[3][3];
        if (!invertMatrix3x3(S, S_inv)) return;

        float K[6][3] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    K[i][j] += PHt[i][k] * S_inv[k][j];

        float correction[6] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 3; j++)
                correction[i] += K[i][j] * y[j];

        roll += correction[0];
        pitch += correction[1];
        yaw += correction[2];
        bgx += correction[3];
        bgy += correction[4];
        bgz += correction[5];

        wrapAngle(roll);
        wrapAngle(pitch);
        wrapAngle(yaw);

        // P = (I - K*H) * P
        float KH[6][6] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 3; k++)
                    KH[i][j] += K[i][k] * H[k][j];

        float P_new[6][6] = {{0}};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                float val = 0;
                for (int k = 0; k < 6; k++)
                    val += ((i == k ? 1.0f : 0.0f) - KH[i][k]) * P[k][j];
                P_new[i][j] = val;
            }

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P[i][j] = P_new[i][j];
    }

    void initialize(float ax, float ay, float az, float mx, float my, float mz) {
        roll = atan2f(ay, az) * RAD2DEG;
        pitch = atan2f(-ax, std::sqrtf(ay * ay + az * az)) * RAD2DEG;

        float sr = std::sinf(roll * DEG2RAD);
        float cr = std::cosf(roll * DEG2RAD);
        float sp = std::sinf(pitch * DEG2RAD);
        float cp = std::cosf(pitch * DEG2RAD);

        float Xh = mx * cp + mz * sp;
        float Yh = mx * sr * sp + my * cr - mz * sr * cp;
        yaw = atan2f(-Yh, Xh) * RAD2DEG;

        bgx = bgy = bgz = 0.0f;

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                P[i][j] = (i == j) ? 1.0f : 0.0f;

        initialized = true;
    }

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getBiasX() const { return bgx; }
    float getBiasY() const { return bgy; }
    float getBiasZ() const { return bgz; }

private:
    void wrapAngle(float& a) const {
        while (a > 180.0f) a -= 360.0f;
        while (a < -180.0f) a += 360.0f;
    }
};

#endif // EKF_H