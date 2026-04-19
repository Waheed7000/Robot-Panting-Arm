#include "config.h"
#include "imu.h"

ImuModule imu;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  imu.begin();
  imu.calibrateGyro();
  imu.setReferencePose();

  Serial.println("IMU ready.");
}

void loop() {
  imu.update();

  Orientation angles = imu.getOrientation();

  // Apply dead zone outside the IMU module
  if (fabs(angles.roll) < DEADZONE_ROLL_DEG) {
    angles.roll = 0.0f;
  }

  if (fabs(angles.pitch) < DEADZONE_PITCH_DEG) {
    angles.pitch = 0.0f;
  }

  Serial.print("Roll: ");
  Serial.print(angles.roll, 2);
  Serial.print("  Pitch: ");
  Serial.println(angles.pitch, 2);

  delay(LOOP_DELAY_MS);
}