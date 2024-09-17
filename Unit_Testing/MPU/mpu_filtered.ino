#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define ALPHA     0.95F

float filteredAngleZ = 0.0;
unsigned long previousTime = 0;

void setup() {
  Serial.begin(115200);
  MpuInit();
}

void loop() {
  filteredAngleZ = getMpuReading();
  
  Serial.print("Mpu Angle : ");
  Serial.println(filteredAngleZ);

  delay(100);
}

void MpuInit(){
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();
  filteredAngleZ = mpu.getAngleZ();
  previousTime = millis();
}

float getMpuReading(){
  mpu.update();
  // Get the current time and calculate the time interval
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Time in seconds
  previousTime = currentTime;
  // Get raw Z angle from the MPU6050
  float rawAngleZ = mpu.getAngleZ();
  // Apply complementary filter:
  // Combine the gyro angle (short-term) and the accelerometer angle (long-term)
  filteredAngleZ = ALPHA * (filteredAngleZ + mpu.getGyroZ() * deltaTime) + (1 - ALPHA) * rawAngleZ;
  return filteredAngleZ;
}
