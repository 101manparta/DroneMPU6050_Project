#ifndef MATRIX_H
#define MATRIX_H

#include <cmath>
#include <iostream>

// Konstanta matematika
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float DEG2RAD = static_cast<float>(M_PI / 180.0);
const float RAD2DEG = static_cast<float>(180.0 / M_PI);

// Vektor 3D
struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }

    float magnitude() const { return std::sqrtf(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float mag = magnitude();
        if (mag == 0) return Vec3(0, 0, 0);
        return Vec3(x / mag, y / mag, z / mag);
    }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 cross(const Vec3& o) const {
        return Vec3(
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        );
    }

    void print(const char* name = "Vec3") const {
        std::cout << name << " = [" << x << ", " << y << ", " << z << "]\n";
    }
};

// Matriks 3x3 (baris-major)
struct Mat3 {
    float m[3][3];

    Mat3() {
        identity();
    }

    void identity() {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
    }

    float* operator[](int i) { return m[i]; }
    const float* operator[](int i) const { return m[i]; }

    void print(const char* name = "Mat3") const {
        std::cout << name << " =\n";
        for (int i = 0; i < 3; i++) {
            std::cout << "  [ " << m[i][0] << " " << m[i][1] << " " << m[i][2] << " ]\n";
        }
    }
};

// Perkalian matriks 3x3
inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
        }
    return r;
}

// Matriks * Vektor
inline Vec3 operator*(const Mat3& a, const Vec3& v) {
    return Vec3(
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z
    );
}

// Matriks rotasi (derajat)
inline Mat3 rotX(float deg) {
    float a = deg * DEG2RAD;
    float c = std::cosf(a), s = std::sinf(a);
    Mat3 r;
    r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

inline Mat3 rotY(float deg) {
    float a = deg * DEG2RAD;
    float c = std::cosf(a), s = std::sinf(a);
    Mat3 r;
    r.m[0][0] = c; r.m[0][2] = s;
    r.m[2][0] = -s; r.m[2][2] = c;
    return r;
}

inline Mat3 rotZ(float deg) {
    float a = deg * DEG2RAD;
    float c = std::cosf(a), s = std::sinf(a);
    Mat3 r;
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

// Matriks rotasi lengkap: R = Rz(yaw) * Ry(pitch) * Rx(roll)
// Mengubah vektor dari world frame ke body frame drone
inline Mat3 attitudeMatrix(float rollDeg, float pitchDeg, float yawDeg) {
    return rotZ(yawDeg) * rotY(pitchDeg) * rotX(rollDeg);
}

#endif // MATRIX_H