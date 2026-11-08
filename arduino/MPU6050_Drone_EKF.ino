/*
 * MPU6050 + HMC5883L Drone Sensor - EKF Ready
 * Project: Sensor IMU Drone dengan Kalkulus, Vektor, Matriks, EKF, Magnetometer
 * 
 * Hardware: Arduino Uno/Nano + MPU6050 (Accel+Gyro) + HMC5883L (Magnetometer)
 * Komunikasi: I2C (Wire)
 * Output: Serial CSV untuk EKF host dan analisis C++
 *
 * Fitur:
 * - Kalibrasi otomatis MPU6050 + HMC5883L
 * - Baca data 9-DOF (accel, gyro, mag)
 * - Tilt-compensated heading untuk yaw absolut
 * - Output CSV lengkap untuk EKF di sisi host
 */

#include <Wire.h>

#define MPU_ADDR 0x68
#define MAG_ADDR 0x1E  // HMC5883L default address

// MPU6050 Registers
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_TEMP_OUT_H   = 0x41;
const uint8_t REG_GYRO_XOUT_H  = 0x43;

// HMC5883L Registers
const uint8_t MAG_REG_CONFIG_A = 0x00;
const uint8_t MAG_REG_CONFIG_B = 0x01;
const uint8_t MAG_REG_MODE     = 0x02;
const uint8_t MAG_REG_DATA_X_M = 0x03;

// Sensitivity (±2g accel, ±250°/s gyro)
const float ACCEL_SENS = 16384.0f;
const float GYRO_SENS  = 131.0f;

// Offset kalibrasi
float axOff = 0, ayOff = 0, azOff = 0;
float gxOff = 0, gyOff = 0, gzOff = 0;

// Magnetometer calibration (hard iron offset + soft iron scale)
float mxOff = 0, myOff = 0, mzOff = 0;
float mxScale = 1, myScale = 1, mzScale = 1;
float mxMin = 1000, mxMax = -1000;
float myMin = 1000, myMax = -1000;
float mzMin = 1000, mzMax = -1000;

// State attitudo
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
unsigned long lastTime = 0;
bool calibrated = false;
bool magCalibrated = false;

const float ALPHA = 0.98f;

// Fungsi bantu I2C
void writeReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

int16_t readReg16(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, (uint8_t)2, (uint8_t)true);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  Wire.setClock(400000);

  // MPU6050 init
  writeReg(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  writeReg(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);
  writeReg(MPU_ADDR, REG_GYRO_CONFIG, 0x00);

  // HMC5883L init
  writeReg(MAG_ADDR, MAG_REG_CONFIG_A, 0x70);  // 8 samples averaged, 15 Hz, normal measurement
  writeReg(MAG_ADDR, MAG_REG_CONFIG_B, 0x20);  // Gain = 1.3 Ga
  writeReg(MAG_ADDR, MAG_REG_MODE, 0x00);      // Continuous measurement mode

  delay(100);

  Serial.println("========================================");
  Serial.println("MPU6050 + HMC5883L Drone Sensor EKF");
  Serial.println("Kalibrasi IMU - JANGAN GERAKKAN SENSOR");
  Serial.println("========================================");

  calibrateMPU();
  calibrated = true;
  lastTime = micros();

  Serial.println("Kalibrasi MPU6050 selesai.");
  Serial.println("Masukkan mode kalibrasi magnetometer (putar drone 360 derajat)");
  Serial.println("timestamp_ms,roll,pitch,yaw,ax,ay,az,gx,gy,gz,mx,my,mz,temp_c,heading_tilt_comp");
}

void calibrateMPU() {
  const int samples = 2000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H);
    int16_t ay = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H + 2);
    int16_t az = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H + 4);
    readReg16(MPU_ADDR, REG_TEMP_OUT_H); // skip temp
    int16_t gx = readReg16(MPU_ADDR, REG_GYRO_XOUT_H);
    int16_t gy = readReg16(MPU_ADDR, REG_GYRO_XOUT_H + 2);
    int16_t gz = readReg16(MPU_ADDR, REG_GYRO_XOUT_H + 4);

    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(1);
  }

  axOff = axSum / (float)samples;
  ayOff = aySum / (float)samples;
  azOff = azSum / (float)samples - ACCEL_SENS;

  gxOff = gxSum / (float)samples;
  gyOff = gySum / (float)samples;
  gzOff = gzSum / (float)samples;
}

void calibrateMagnetometer() {
  Serial.println("Putar drone dalam semua arah selama 30 detik...");
  delay(2000);

  unsigned long start = millis();
  while (millis() - start < 30000) {
    int16_t mxRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M);
    int16_t mzRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M + 4);
    int16_t myRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M + 2);

    if (mxRaw < mxMin) mxMin = mxRaw;
    if (mxRaw > mxMax) mxMax = mxRaw;
    if (myRaw < myMin) myMin = myRaw;
    if (myRaw > myMax) myMax = myRaw;
    if (mzRaw < mzMin) mzMin = mzRaw;
    if (mzRaw > mzMax) mzMax = mzRaw;

    delay(50);
  }

  mxOff = (mxMax + mxMin) / 2.0f;
  myOff = (myMax + myMin) / 2.0f;
  mzOff = (mzMax + mzMin) / 2.0f;

  float mxRange = mxMax - mxMin;
  float myRange = myMax - myMin;
  float mzRange = mzMax - mzMin;
  float avgRange = (mxRange + myRange + mzRange) / 3.0f;

  mxScale = avgRange / mxRange;
  myScale = avgRange / myRange;
  mzScale = avgRange / mzRange;

  magCalibrated = true;
  Serial.println("Kalibrasi magnetometer selesai.");
  Serial.print("mxOff="); Serial.print(mxOff); Serial.print(" myOff="); Serial.print(myOff);
  Serial.print(" mzOff="); Serial.println(mzOff);
}

void loop() {
  if (!calibrated) return;

  // Baca MPU6050
  int16_t axRaw = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H);
  int16_t ayRaw = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H + 2);
  int16_t azRaw = readReg16(MPU_ADDR, REG_ACCEL_XOUT_H + 4);
  int16_t tempRaw = readReg16(MPU_ADDR, REG_TEMP_OUT_H);
  int16_t gxRaw = readReg16(MPU_ADDR, REG_GYRO_XOUT_H);
  int16_t gyRaw = readReg16(MPU_ADDR, REG_GYRO_XOUT_H + 2);
  int16_t gzRaw = readReg16(MPU_ADDR, REG_GYRO_XOUT_H + 4);

  // Konversi MPU6050
  float ax = (axRaw - axOff) / ACCEL_SENS;
  float ay = (ayRaw - ayOff) / ACCEL_SENS;
  float az = (azRaw - azOff) / ACCEL_SENS;
  float gx = (gxRaw - gxOff) / GYRO_SENS;
  float gy = (gyRaw - gyOff) / GYRO_SENS;
  float gz = (gzRaw - gzOff) / GYRO_SENS;
  float temp = tempRaw / 340.0f + 36.53f;

  // Baca Magnetometer
  int16_t mxRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M);
  int16_t mzRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M + 4);
  int16_t myRaw = readReg16(MAG_ADDR, MAG_REG_DATA_X_M + 2);

  // Apply magnetometer calibration
  float mx = (mxRaw - mxOff) * mxScale;
  float my = (myRaw - myOff) * myScale;
  float mz = (mzRaw - mzOff) * mzScale;

  // Kalkulus: integrasi waktu untuk sudut (gyro)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  // Sudut dari accelerometer
  float accRoll  = atan2f(ay, az) * 180.0f / PI;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // Complementary filter (fallback jika mag tidak aktif)
  roll  = ALPHA * roll  + (1.0f - ALPHA) * accRoll;
  pitch = ALPHA * pitch + (1.0f - ALPHA) * accPitch;

  // Tilt-compensated heading dari magnetometer (jika terkalibrasi)
  float heading = yaw; // default dari gyro
  if (magCalibrated) {
    // Tilt compensation
    float cosRoll = cosf(roll * DEG2RAD);
    float sinRoll = sinf(roll * DEG2RAD);
    float cosPitch = cosf(pitch * DEG2RAD);
    float sinPitch = sinf(pitch * DEG2RAD);

    float Xh = mx * cosPitch + mz * sinPitch;
    float Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
    heading = atan2f(-Yh, Xh) * 180.0f / PI;

    // Normalisasi heading
    while (heading > 180.0f) heading -= 360.0f;
    while (heading < -180.0f) heading += 360.0f;

    // Gabungkan yaw gyro dengan heading mag (sederhana)
    yaw = 0.90f * yaw + 0.10f * heading;
  }

  // Normalisasi sudut
  while (roll > 180.0f) roll -= 360.0f;
  while (roll < -180.0f) roll += 360.0f;
  while (pitch > 180.0f) pitch -= 360.0f;
  while (pitch < -180.0f) pitch += 360.0f;
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;

  // Output CSV untuk EKF host
  Serial.print(now / 1000.0f); Serial.print(",");
  Serial.print(roll, 4); Serial.print(",");
  Serial.print(pitch, 4); Serial.print(",");
  Serial.print(yaw, 4); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.print(mx, 4); Serial.print(",");
  Serial.print(my, 4); Serial.print(",");
  Serial.print(mz, 4); Serial.print(",");
  Serial.print(temp, 2); Serial.print(",");
  Serial.println(heading, 4);

  delay(10); // 100 Hz
}