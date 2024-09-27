#define MOTOR_R_1       PA0
#define MOTOR_R_2       PA1
#define MOTOR_L_1       PB0
#define MOTOR_L_2       PB1

void motorInit(void)
{
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
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

void setup() {
  motorInit();
  delay(10);

}

void loop() {
  moveForward();
}
