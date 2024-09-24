#include "VLX_Interface.h"

Adafruit_VL6180X sensors[COUNT_SENSORS];

void VLX_setupSensors() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  for (uint8_t i; i<COUNT_SENSORS; i++) {
    sensors[i] = Adafruit_VL6180X();
  }

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  //initing LOX1
  if (!sensors[RIGHT_VLX].begin()) {
    display.print("Right Sensor Failed!");
  }
  sensors[RIGHT_VLX].setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initing LOX2
  if (!sensors[MIDDLE_VLX].begin()) {
    display.print("Middle Sensor Failed!");
  }
  sensors[MIDDLE_VLX].setAddress(LOX2_ADDRESS);
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!sensors[LEFT_VLX].begin()) {
    display.print("Left Sensor Failed!");
  }
  sensors[LEFT_VLX].setAddress(LOX3_ADDRESS);
  delay(10);
}

uint8_t VLX_readDistance(uint8_t sensorID) {
    return (sensors[sensorID].readRange()/10);
}
