#include "N20Motor.h"

static volatile uint32_t encoderA_counter = 0;
static volatile uint32_t encoderB_counter = 0;

void motorInit(void)
{
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
    attachInterrupt(MOTOR_R_ENC_A, rightEncoderCb, CHANGE);
    attachInterrupt(MOTOR_R_ENC_B, rightEncoderCb, CHANGE);
}
void moveForward()
{
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
}
void stop()
{
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, LOW);
}
void moveBackward()
{
    digitalWrite(MOTOR_R_2, HIGH);
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    digitalWrite(MOTOR_L_1, LOW);
}
void rightEncoderCb(void)
{
    uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
    stateA == stateB ? encoderB_counter++ : encoderB_counter--;
}
void leftEncoderCb(void)
{
    uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
    stateA == stateB ? encoderB_counter++ : encoderB_counter--;
}
