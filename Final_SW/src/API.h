#ifndef API_H
#define API_H

#include "VLX_Interface.h"
#include "KalmanMPU6050.h"
#include "OLED_Interface.h"

void HW_Init();
bool wallFront();
bool wallRight();
bool wallLeft();
void center();
void moveForward();
void turnRight();
void turnLeft();

int orientation(int orient, char turning);
void updateCoordinates(int orient, int *new_x, int *new_y);


#endif