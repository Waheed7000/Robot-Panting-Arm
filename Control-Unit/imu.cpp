#include "imu.h"
#include "config.h"

// =========================
// Public methods
// =========================

void ImuModule::begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // =========================
  // Configure accelerometer range (+/-2g)
  // =========================
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);   // ACCEL_CONFIG register
  Wire.write(0x00);   // +/-2g
  Wire.endTransmission();

  // =========================
  // Configure gyroscope range (+/-250 deg/s)
  // =========================
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_GYRO_CONFIG);   // GYRO_CONFIG register
  Wire.write(0x00);   // +/-250 deg/s
  Wire.endTransmission();

  delay(100);

  lastUpdateMicros = micros();
}

void ImuModule::calibrateGyro() {
  long sumGx = 0;
  long sumGy = 0;
  long sumGz = 0;

  Serial.println("Keep the controller still. Calibrating gyro...");

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    RawImuData raw;
    if (!readRawData(raw)) {
      i--; // retry this sample
      continue;
    }

    sumGx += raw.gx;
    sumGy += raw.gy;
    sumGz += raw.gz;

    delay(GYRO_CALIBRATION_DELAY_MS);
  }

  gyroBiasX = static_cast<float>(sumGx) / GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = static_cast<float>(sumGy) / GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = static_cast<float>(sumGz) / GYRO_CALIBRATION_SAMPLES;

  Serial.println("Gyro calibration done.");
}

void ImuModule::setReferencePose() {
  // Take several filtered updates so the reference pose is stable
  for (int i = 0; i < 100; i++) {
    update();
    delay(5);
  }

  rollReferenceDeg = rollDeg;
  pitchReferenceDeg = pitchDeg;

  Serial.println("Reference pose captured.");
}

void ImuModule::update() {
  float dt = computeDeltaTime();

  RawImuData raw;
  if (!readRawData(raw)) {
    return; // skip this loop if read failed
  }

  // Remove gyro bias before scaling
  raw.gx = static_cast<int16_t>(raw.gx - gyroBiasX);
  raw.gy = static_cast<int16_t>(raw.gy - gyroBiasY);
  raw.gz = static_cast<int16_t>(raw.gz - gyroBiasZ);

  ScaledImuData data = scaleRawData(raw);
  applyAccelLowPassFilter(data);

  float rollAccDeg = 0.0f;
  float pitchAccDeg = 0.0f;
  computeAccelAngles(data, rollAccDeg, pitchAccDeg);

  // Integrate gyro rates
  float rollGyroDeg = rollDeg + data.gx_dps * dt;
  float pitchGyroDeg = pitchDeg + data.gy_dps * dt;

  // Complementary filter:
  // gyro gives smooth short-term motion
  // accelerometer corrects long-term drift
  rollDeg = COMPLEMENTARY_ALPHA * rollGyroDeg +
            (1.0f - COMPLEMENTARY_ALPHA) * rollAccDeg;

  pitchDeg = COMPLEMENTARY_ALPHA * pitchGyroDeg +
             (1.0f - COMPLEMENTARY_ALPHA) * pitchAccDeg;
}

Orientation ImuModule::getOrientation() const {
  Orientation result;
  result.roll = rollDeg - rollReferenceDeg;
  result.pitch = pitchDeg - pitchReferenceDeg;
  return result;
}

// =========================
// Private methods
// =========================

bool ImuModule::readRawData(RawImuData& raw) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesRead = Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

  if (bytesRead != 14) {
    return false;
  }

  raw.ax = (Wire.read() << 8) | Wire.read();
  raw.ay = (Wire.read() << 8) | Wire.read();
  raw.az = (Wire.read() << 8) | Wire.read();

  // skip temperature
  Wire.read();
  Wire.read();

  raw.gx = (Wire.read() << 8) | Wire.read();
  raw.gy = (Wire.read() << 8) | Wire.read();
  raw.gz = (Wire.read() << 8) | Wire.read();

  return true;
}

ScaledImuData ImuModule::scaleRawData(const RawImuData& raw) {
  ScaledImuData data{};

  data.ax_g = raw.ax / ACCEL_SCALE_LSB_PER_G;
  data.ay_g = raw.ay / ACCEL_SCALE_LSB_PER_G;
  data.az_g = raw.az / ACCEL_SCALE_LSB_PER_G;

  data.gx_dps = raw.gx / GYRO_SCALE_LSB_PER_DPS;
  data.gy_dps = raw.gy / GYRO_SCALE_LSB_PER_DPS;
  data.gz_dps = raw.gz / GYRO_SCALE_LSB_PER_DPS;

  return data;
}

void ImuModule::applyAccelLowPassFilter(ScaledImuData& data) {
  if (!accelFilterInitialized) {
    axFiltered_g = data.ax_g;
    ayFiltered_g = data.ay_g;
    azFiltered_g = data.az_g;
    accelFilterInitialized = true;
  } else {
    axFiltered_g = ACCEL_LPF_ALPHA * data.ax_g + (1.0f - ACCEL_LPF_ALPHA) * axFiltered_g;
    ayFiltered_g = ACCEL_LPF_ALPHA * data.ay_g + (1.0f - ACCEL_LPF_ALPHA) * ayFiltered_g;
    azFiltered_g = ACCEL_LPF_ALPHA * data.az_g + (1.0f - ACCEL_LPF_ALPHA) * azFiltered_g;
  }

  data.ax_g = axFiltered_g;
  data.ay_g = ayFiltered_g;
  data.az_g = azFiltered_g;
}

void ImuModule::computeAccelAngles(const ScaledImuData& data, float& rollAccDeg, float& pitchAccDeg) {
  rollAccDeg = atan2(data.ay_g, sqrt(data.ax_g * data.ax_g + data.az_g * data.az_g)) * 180.0f / PI;
  pitchAccDeg = atan2(-data.ax_g, sqrt(data.ay_g * data.ay_g + data.az_g * data.az_g)) * 180.0f / PI;
}

float ImuModule::computeDeltaTime() {
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
  lastUpdateMicros = nowMicros;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f;
  }

  return dt;
}