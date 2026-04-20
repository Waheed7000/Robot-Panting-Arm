#include "config.h"
#include "imu.h"

ImuModule imu;

MotionCommand mapOrientationToCommand(const Orientation& angles) {
  MotionCommand cmd;

  cmd.yawCmd = angles.roll / MAX_CONTROL_ANGLE_DEG;
  cmd.pitchCmd = angles.pitch / MAX_CONTROL_ANGLE_DEG;

  cmd.yawCmd = constrain(cmd.yawCmd, -1.0f, 1.0f);
  cmd.pitchCmd = constrain(cmd.pitchCmd, -1.0f, 1.0f);

  return cmd;
}

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

  MotionCommand cmd = mapOrientationToCommand(angles);

  Serial.print("Roll: ");
  Serial.print(angles.roll, 2);
  Serial.print("  Pitch: ");
  Serial.print(angles.pitch, 2);

  Serial.print("  |  YawCmd: ");
  Serial.print(cmd.yawCmd, 2);
  Serial.print("  PitchCmd: ");
  Serial.println(cmd.pitchCmd, 2);

  delay(LOOP_DELAY_MS);
}