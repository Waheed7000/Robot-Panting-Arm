#include "config.h"
#include "imu.h"
#include "comm_link.h"

ImuModule imu;
CommLink comm;

static const uint32_t COMM_SEND_INTERVAL_MS = 50;
uint32_t lastSendMs = 0;

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

  // LED ON → calibration starting
  pinMode(CALIB_LED_PIN, OUTPUT);
  digitalWrite(CALIB_LED_PIN, HIGH);

  imu.begin();
  imu.calibrateGyro();
  imu.setReferencePose();

  digitalWrite(CALIB_LED_PIN, LOW);

  Serial.println("IMU ready.");

  comm.begin();
}

void loop() {
  comm.update();

  imu.update();

  Orientation angles = imu.getOrientation();

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

  uint32_t now = millis();
  if (now - lastSendMs >= COMM_SEND_INTERVAL_MS) {
    lastSendMs = now;

    ControlPacketData packet;
    packet.roll = angles.roll;
    packet.pitch = angles.pitch;
    packet.yawCmd = cmd.yawCmd;
    packet.pitchCmd = cmd.pitchCmd;

    comm.sendControlData(packet);
  }

  delay(LOOP_DELAY_MS);
}
