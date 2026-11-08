/*
 * MPU6050 Drone Sensor - X, Y, Z Attitude Monitoring
 * Project: Sensor IMU Drone dengan Kalkulus, Vektor & Matriks, Akurasi dan Presisi
 * 
 * Hardware: Arduino Uno/Nano + MPU6050 (GY-521)
 * Komunikasi: I2C (Wire)
 * Output: Serial CSV untuk diolah oleh program C++ host
 *
 * Fitur:
 * - Kalibrasi otomatis offset accelerometer & gyro
 * - Integrasi waktu (kalkulus) untuk estimasi sudut dari gyro
 * - Complementary filter (gabung accel + gyro) untuk mengurangi drift
 * - Matriks rotasi siap digunakan di sisi C++ host
 * - Output timestamp untuk analisis kecepatan & time-of-flight
 */

#include <Wire.h>

#define MPU_ADDR 0x68

// Register MPU6050
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_TEMP_OUT_H   = 0x41;
const uint8_t REG_GYRO_XOUT_H  = 0x43;

// Sensitivity (±2g accel, ±250°/s gyro)
const float ACCEL_SENS = 16384.0f;   // LSB per g
const float GYRO_SENS  = 131.0f;     // LSB per deg/s

// Offset kalibrasi
float axOff = 0, ayOff = 0, azOff = 0;
float gxOff = 0, gyOff = 0, gzOff = 0;

// State attitudo
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
unsigned long lastTime = 0;
bool calibrated = false;

const float ALPHA = 0.98f; // Complementary filter coefficient

// Tulis 1 byte ke register
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// Baca 16-bit signed dari register (big-endian)
int16_t readReg16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Tunggu serial ready (untuk beberapa board)

  Wire.begin();
  Wire.setClock(400000); // I2C 400kHz

  // Wake up MPU6050
  writeReg(REG_PWR_MGMT_1, 0x00);
  // Accel ±2g
  writeReg(REG_ACCEL_CONFIG, 0x00);
  // Gyro ±250 deg/s
  writeReg(REG_GYRO_CONFIG, 0x00);

  delay(100);

  Serial.println("========================================");
  Serial.println("MPU6050 Drone Sensor Project");
  Serial.println("Kalibrasi dimulai - JANGAN GERAKKAN SENSOR");
  Serial.println("========================================");

  calibrate();
  calibrated = true;
  lastTime = micros();

  Serial.println("Kalibrasi selesai.");
  Serial.println("timestamp_ms,roll,pitch,yaw,ax,ay,az,gx,gy,gz,temp_c");
}

void calibrate() {
  const int samples = 2000;
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax = readReg16(REG_ACCEL_XOUT_H);
    int16_t ay = readReg16(REG_ACCEL_XOUT_H + 2);
    int16_t az = readReg16(REG_ACCEL_XOUT_H + 4);
    // skip temp
    int16_t gx = readReg16(REG_GYRO_XOUT_H);
    int16_t gy = readReg16(REG_GYRO_XOUT_H + 2);
    int16_t gz = readReg16(REG_GYRO_XOUT_H + 4);

    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;

    delay(1);
  }

  axOff = axSum / (float)samples;
  ayOff = aySum / (float)samples;
  azOff = azSum / (float)samples - ACCEL_SENS; // Buang gravitasi 1g pada Z

  gxOff = gxSum / (float)samples;
  gyOff = gySum / (float)samples;
  gzOff = gzSum / (float)samples;
}

void loop() {
  if (!calibrated) return;

  // Baca data mentah
  int16_t axRaw = readReg16(REG_ACCEL_XOUT_H);
  int16_t ayRaw = readReg16(REG_ACCEL_XOUT_H + 2);
  int16_t azRaw = readReg16(REG_ACCEL_XOUT_H + 4);
  int16_t tempRaw = readReg16(REG_TEMP_OUT_H);
  int16_t gxRaw = readReg16(REG_GYRO_XOUT_H);
  int16_t gyRaw = readReg16(REG_GYRO_XOUT_H + 2);
  int16_t gzRaw = readReg16(REG_GYRO_XOUT_H + 4);

  // Konversi ke unit fisik setelah kalibrasi
  float ax = (axRaw - axOff) / ACCEL_SENS;
  float ay = (ayRaw - ayOff) / ACCEL_SENS;
  float az = (azRaw - azOff) / ACCEL_SENS;
  float gx = (gxRaw - gxOff) / GYRO_SENS;   // deg/s
  float gy = (gyRaw - gyOff) / GYRO_SENS;   // deg/s
  float gz = (gzRaw - gzOff) / GYRO_SENS;   // deg/s
  float temp = tempRaw / 340.0f + 36.53f;   // Celsius

  // Kalkulus: integrasi waktu untuk sudut (gyro)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f; // detik
  lastTime = now;

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  // Sudut dari accelerometer (referensi gravitasi)
  float accRoll  = atan2f(ay, az) * 180.0f / PI;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // Complementary filter: gabung gyro (stabil jangka pendek) + accel (stabil jangka panjang)
  roll  = ALPHA * roll  + (1.0f - ALPHA) * accRoll;
  pitch = ALPHA * pitch + (1.0f - ALPHA) * accPitch;
  // Yaw tetap dari integrasi gyro (butuh magnetometer untuk referensi absolut)

  // Normalisasi sudut ke [-180, 180]
  while (roll  > 180.0f) roll  -= 360.0f;
  while (roll  < -180.0f) roll  += 360.0f;
  while (pitch > 180.0f) pitch -= 360.0f;
  while (pitch < -180.0f) pitch += 360.0f;
  while (yaw   > 180.0f) yaw   -= 360.0f;
  while (yaw   < -180.0f) yaw   += 360.0f;

  // Output CSV ke Serial (untuk dibaca C++ host / Python)
  Serial.print(now / 1000.0f); Serial.print(",");   // timestamp ms
  Serial.print(roll, 4); Serial.print(",");
  Serial.print(pitch, 4); Serial.print(",");
  Serial.print(yaw, 4); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.println(temp, 2);

  delay(10); // 100 Hz sampling
}