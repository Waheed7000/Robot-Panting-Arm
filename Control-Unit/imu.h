#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

struct RawImuData {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct ScaledImuData {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
};

struct Orientation {
  float roll;
  float pitch;
};

class ImuModule {
public:
  void begin();
  void calibrateGyro();
  void setReferencePose();
  void update();
  Orientation getOrientation() const;

private:
  bool readRawData(RawImuData& raw);
  ScaledImuData scaleRawData(const RawImuData& raw);
  void applyAccelLowPassFilter(ScaledImuData& data);
  void computeAccelAngles(const ScaledImuData& data, float& rollAccDeg, float& pitchAccDeg);
  float computeDeltaTime();

private:
  // Gyro bias in raw units
  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

  // Filtered accelerometer values in g
  float axFiltered_g = 0.0f;
  float ayFiltered_g = 0.0f;
  float azFiltered_g = 0.0f;

  bool accelFilterInitialized = false;

  // Filtered absolute orientation
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;

  // Reference offsets to make startup pose = zero
  float rollReferenceDeg = 0.0f;
  float pitchReferenceDeg = 0.0f;

  // Timing
  uint32_t lastUpdateMicros = 0;
};

#endif