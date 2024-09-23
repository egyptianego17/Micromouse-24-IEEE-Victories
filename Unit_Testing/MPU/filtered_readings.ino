#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

float filteredAngleZ = 0.0; // Initial filtered angle
float alpha = 0.95;         // Complementary filter coefficient
unsigned long previousTime = 0;  // To store the previous time for interval calculation

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();  // Gyro and accelerometer offsets

  // Initialize the complementary filter with the initial raw angle
  filteredAngleZ = mpu.getAngleZ();
  previousTime = millis();  // Initialize the previous time
}

void loop() {
  mpu.update();

  // Get the current time and calculate the time interval
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Time in seconds
  previousTime = currentTime;

  // Get raw Z angle from the MPU6050
  float rawAngleZ = mpu.getAngleZ();
  
  // Apply complementary filter:
  // Combine the gyro angle (short-term) and the accelerometer angle (long-term)
  filteredAngleZ = alpha * (filteredAngleZ + mpu.getGyroZ() * deltaTime) + (1 - alpha) * rawAngleZ;

  // Print the raw and filtered angle values
  Serial.print("Raw Z : ");
  Serial.print(rawAngleZ);
  Serial.print("\tFiltered Z : ");
  Serial.println(filteredAngleZ);

  delay(100);
}
