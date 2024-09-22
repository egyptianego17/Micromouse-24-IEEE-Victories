#include "VLX_Interface.h"

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
uint16_t
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
    //  OLED
  }
  sensors[RIGHT_VLX].setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initing LOX2
  if (!sensors[Middle_VLX].begin()) {
    //OLED
  }
  sensors[Middle_VLX].setAddress(LOX2_ADDRESS);
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!sensors[LEFT_VLX].begin()) {
    //OLED
  }
  sensors[LEFT_VLX].setAddress(LOX3_ADDRESS);
  delay(10);
}

uint8_t readDistance(Adafruit_VL6180X &vl){
    return vl.readRange();
}
