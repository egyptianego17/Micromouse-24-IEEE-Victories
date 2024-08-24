#include "../lib/main.hpp"

/* VL6180X sensor object */
VL6180X sensor;

void setup() 
{
  Serial2.begin(115200);
  Serial2.write("Adafruit VL6180x test!\n");

  /* Initialize the sensor */
  vl6SensorConfig();

  pinMode(ON_BOARD_LED, OUTPUT);
}

void loop() 
{
  Serial2.print(sensor.readRangeContinuousMillimeters() / 10.0);
  Serial2.print(" cm\n");
}

void vl6SensorConfig(void)
{
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor.setTimeout(500);
   // stop continuous mode if already active
  sensor.stopContinuous();
  delay(300);
  sensor.startInterleavedContinuous(100);

  Serial2.write("Sensor found!\n");
}
