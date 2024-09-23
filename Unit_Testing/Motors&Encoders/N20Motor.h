#ifndef N20_MOTOR_H
#define N20_MOTOR_H

#include <Arduino.h>

#define MOTOR_R_1       PA0
#define MOTOR_R_2       PA1
#define MOTOR_L_1       PB0
#define MOTOR_L_2       PB1

#define MOTOR_R_ENC_A   0
#define MOTOR_R_ENC_B   0

#define MOTOR_L_ENC_A   0
#define MOTOR_L_ENC_B   0

void motorInit(void);
void rightEncoderCb(void);
void leftEncoderCb(void);
void moveForward(void);
void stop(void);
void moveBackward(void);

#endif /* N20_MOTOR_H */
