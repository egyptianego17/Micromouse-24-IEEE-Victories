#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // Read data from MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
  
  // // Calculate accelerometer angles (in degrees)
  // float accelAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  // float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  
  // Print data to the serial monitor
  //Serial.print("Accel X: "); Serial.print(ax); Serial.print("\tAccel Y: "); Serial.print(ay); Serial.print("\tAccel Z: "); Serial.println(az);
  Serial.print("Gyro X: "); Serial.print(gyroX); Serial.print("\tGyro Y: "); Serial.print(gyroY); Serial.print("\tGyro Z: "); Serial.println(gyroZ);
  //Serial.print("Accel Angle X: "); Serial.print(accelAngleX); Serial.print("\tAccel Angle Y: "); Serial.println(accelAngleY);
  // Serial.println();

  delay(1000); // Wait for a moment before taking another reading
}