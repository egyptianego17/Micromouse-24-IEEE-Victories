#ifndef VLX_INTERFACE_H
#define VLX_INTERFACE_H

#include <Adafruit_VL6180X.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 PB2      //Right
#define SHT_LOX2 PC14     //Middle
#define SHT_LOX3 PC15     //Left

#define COUNT_SENSORS 3

#define RIGHT_VLX 0
#define Middle_VLX 1
#define LEFT_VLX 2

String sensorsNames[3] = { "Right_Sensor", "Middle_Sensor", "Left_Sensor" };

Adafruit_VL6180X sensors[COUNT_SENSORS];

void sensorsInit();
uint8_t readDistance(Adafruit_VL6180X &vl);

#endif