#include <Wire.h>

#define MPU_ADDR 0x68

int16_t AccX, AccY, AccZ;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // power management
  Wire.write(0);    // wake up
  Wire.endTransmission();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // start from Accel X
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6); // X, Y, Z

  AccX = (Wire.read() << 8) | Wire.read();
  AccY = (Wire.read() << 8) | Wire.read();
  AccZ = (Wire.read() << 8) | Wire.read();

  Serial.print("X: "); Serial.print(AccX);
  Serial.print("  Y: "); Serial.print(AccY);
  Serial.print("  Z: "); Serial.println(AccZ);

  delay(500);
}