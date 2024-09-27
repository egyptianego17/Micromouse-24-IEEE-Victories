#include <sys/_stdint.h>
#include "WSerial.h"
#include "wiring_time.h"
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
    OLED_displayData(0,0,2,"RS Failed!",-1);
  }
  sensors[RIGHT_VLX].setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initing LOX2
  if (!sensors[MIDDLE_VLX].begin()) {
    OLED_displayData(0,22,2,"MS Failed!",-1);
  }
  sensors[MIDDLE_VLX].setAddress(LOX2_ADDRESS);
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!sensors[LEFT_VLX].begin()) {
    OLED_displayData(0,44,2,"LS Failed!",-1);
  }
  sensors[LEFT_VLX].setAddress(LOX3_ADDRESS);
  delay(10);
  delay(1000);
  display.clearDisplay();
}

float VLX_readDistance(uint8_t sensorID) {
  if(sensors[sensorID].begin())
      return (sensors[sensorID].readRange()/10.0);
  else
      return MAX_DISTANCE;
}
