#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(); // Gyro and accelerometer offsets
}

void loop() {
  mpu.update();
  float rawAngle = mpu.getAngleZ();
  Serial.print("\tZ : ");
  Serial.println(rawAngle);
  delay(100);
}