#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================
// General configuration
// =========================
constexpr uint32_t SERIAL_BAUD = 115200;

// =========================
// I2C / MPU6050 configuration
// =========================
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t MPU6050_ADDR = 0x68;

// MPU6050 register addresses
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG  = 0x1B;

// =========================
// Sensor scaling constants
// =========================
// Accelerometer sensitivity for +/-2g
constexpr float ACCEL_SCALE_LSB_PER_G = 16384.0f;

// Gyroscope sensitivity for +/-250 deg/s
constexpr float GYRO_SCALE_LSB_PER_DPS = 131.0f;

// =========================
// Calibration
// =========================
constexpr int GYRO_CALIBRATION_SAMPLES = 1000;
constexpr int GYRO_CALIBRATION_DELAY_MS = 2;


// =========================
// Accelerometer low-pass filter
// =========================
constexpr float ACCEL_LPF_ALPHA = 0.2f;

// =========================
// Complementary filter
// =========================
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// =========================
// Dead zone
// =========================
constexpr float DEADZONE_ROLL_DEG = 2.0f;
constexpr float DEADZONE_PITCH_DEG = 2.0f;

// =========================
// Control mapping
// =========================
constexpr float MAX_CONTROL_ANGLE_DEG = 25.0f;

// =========================
// Loop timing
// =========================
constexpr int LOOP_DELAY_MS = 10;

#endif