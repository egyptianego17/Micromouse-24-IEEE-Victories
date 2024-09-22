#include "VLX_Interface.h"


String sensorsNames[3] = { "Right_Sensor", "Middle_Sensor", "Left_Sensor" };

void sensorsInit() {

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
    displayData(0,0,1,"Sensor Right Failed",-1);
  }
  sensors[RIGHT_VLX].setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initing LOX2
  if (!sensors[Middle_VLX].begin()) {
    displayData(0,22,1,"Sensor Middle Failed",-1);
  }
  sensors[Middle_VLX].setAddress(LOX2_ADDRESS);
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!sensors[LEFT_VLX].begin()) {
    displayData(0,44,1,"Sensor Left Failed",-1);
  }
  sensors[LEFT_VLX].setAddress(LOX3_ADDRESS);
  delay(10);
}

uint8_t readDistance(Adafruit_VL6180X &vl){
    return vl.readRange();
}
