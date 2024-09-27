#include "wiring_time.h"
#include "WSerial.h"
#include "API.h"

// Motor Pins:
#define MOTOR_R_1 PA7
#define MOTOR_R_2 PB14
#define MOTOR_L_1 PB9
#define MOTOR_L_2 PB8

#define MOTOR_R_ENC_A PA2
#define MOTOR_R_ENC_B PA3

#define MOTOR_L_ENC_A PA4
#define MOTOR_L_ENC_B PA5

#define melodyPin PA6

#define CELL_WIDTH (18.0)
/* -------------------------------------------------------- */
void Motors_moveForward(int speed);
void stopRightMotor();
void moveLeftMotor(int speed);
void stopLeftMotor();
void moveRightMotor(int speed);
void stopMotors(void);
void synchronizeMotors(int base_speed);
inline float calculate_distance(uint32_t pulse_count);
/* ------------------ Turning the car ------------------ */
#define MOTOR_DEFAULT_SPEED_R 80
#define MOTOR_DEFAULT_SPEED_L 75

#define ROTATE_ANGLE 90

volatile float currentYaw = 0.0;
// TaskHandle_t updateAngleTaskHandle;
// TaskHandle_t updateWallsDistanceTaskHandle;

double targetYaw = 90.0;
double yaw = 0;
double rateOfIncrease = 0;
bool initialized = false;
double totalYawIncrease = 0;
double desiredAngle = 0;
long previousTime = 0;
long initStartTime = 0;
long initDuration = 10000;

void initGyroRate();
void updateAngleTask(void *pvParameters);
/* ------------------ Walls Detection ------------------ */
float rightDistance = 0;
float leftDistance = 0;
float midDistance = 0;

/* ------------------ Moving the car ------------------ */

const int TOLERANCE = 1;  // Step difference tolerance between encoders
#define TICKS_PER_REV 262.0

// Wheel and encoder constants
const float wheelCircumference = 13.188;                             // cm
const float distance_per_tick = wheelCircumference / TICKS_PER_REV;  // cm per encoder tick (assuming 200 ticks per revolution)

// PID constants
float Kp = 5;

// PID variables
float error = 0.0;
float previous_error = 0.0;
float control_output = 0.0;

// Encoder counters
volatile double encoderA_counter = 0;  // Right encoder
volatile double encoderB_counter = 0;  // Left encoder
const int MIN_SPEED = 70;
const int MAX_SPEED = 90;
float target_distance = 0;

void rightEncoderCb();
void leftEncoderCb();

void HW_Init() {
  Serial.begin(9600);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);

  pinMode(MOTOR_R_ENC_A, INPUT);
  pinMode(MOTOR_R_ENC_B, INPUT);
  pinMode(MOTOR_L_ENC_A, INPUT);
  pinMode(MOTOR_L_ENC_B, INPUT);
  pinMode(PA6, OUTPUT);//buzzer

  // Set up interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, RISING);

  OLED_setup();
  VLX_setupSensors();
  OLED_drawTuffy();

  IMU::init();
  IMU::read();

  initGyroRate();
  
  // xTaskCreate(updateAngleTask,
  //             "Task1",
  //             configMINIMAL_STACK_SIZE + 100,
  //             NULL,
  //             2,
  //             &updateAngleTaskHandle);


  // vTaskStartScheduler();
}

// void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
//   // Handle stack overflow here
//   Serial.println("Stack Overflow!");
//   for (;;)
//     ;
// }

// void vApplicationMallocFailedHook(void) {
//   // Handle malloc failure here
//   Serial.println("Malloc Failed!");
//   for (;;)
//     ;
// }

// void updateWallsDistance(void)
// {
//     rightDistance = VLX_readDistance(RIGHT_VLX);
//     leftDistance = VLX_readDistance(LEFT_VLX);
//     midDistance = VLX_readDistance(MIDDLE_VLX);
//     Serial.print(rightDistance);
//     Serial.print("        ");
//     Serial.print(leftDistance);
//     Serial.print("        ");
//     Serial.println(midDistance);
// }

void center(void)
{
  analogWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, 120);
  analogWrite(MOTOR_L_1, 0);
  digitalWrite(MOTOR_L_2, 120);
  delay(1000);
  Serial.println("Enter Dumb Center..");
  stopMotors();
  delay(200);
  encoderA_counter = 0;
  encoderB_counter = 0;
  target_distance = 4;
  // Calculate the current distance traveled
  float current_distance = 0;
  // Check if the car has reached the target distance
  while (current_distance < target_distance) {
    Serial.println(current_distance);
    // Serial.println(wallRight());
    // Serial.println(wallLeft());
    // Serial.println(wallFront());
    // Calculate the PID error based on target distance and current distance
    error = target_distance - current_distance;
    // Compute PID output for motor speed
    control_output = Kp * error;
    control_output = constrain(control_output, MIN_SPEED, MAX_SPEED);
    // Synchronize the motors based on encoder values and apply the PID-controlled speed
    synchronizeMotors(control_output-25);
    current_distance = calculate_distance(((long int)encoderA_counter + (long int)encoderB_counter) / 2);
  }
  stopMotors();
  encoderA_counter = 0;
  encoderB_counter = 0;
  Serial.println("Exit MoveForward..");
}

void synchronizeMotors(int base_speed) {
  if (abs((long int)encoderA_counter - (long int)encoderB_counter) <= TOLERANCE) {
    Motors_moveForward(base_speed);
  } else if ((long int)encoderA_counter > (long int)encoderB_counter) {
    stopRightMotor();
    moveLeftMotor(base_speed);
  } else if ((long int)encoderB_counter > (long int)encoderA_counter) {
    stopLeftMotor();
    moveRightMotor(base_speed);
  }
}
void Motors_moveForward(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}
void stopRightMotor() {
  analogWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, LOW);
}
// Function to stop only the left motor
void stopLeftMotor() {
  analogWrite(MOTOR_L_1, 0);
  digitalWrite(MOTOR_L_2, LOW);
}
void moveRightMotor(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
}
// Function to move only the left motor
void moveLeftMotor(int speed) {
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}
inline float calculate_distance(uint32_t pulse_count) {
  return ((float)(pulse_count) / TICKS_PER_REV) * wheelCircumference;
}
void stopMotors() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, 0);
}
void Motors_turnLeft() {
  analogWrite(MOTOR_R_1, MOTOR_DEFAULT_SPEED_R);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_2, MOTOR_DEFAULT_SPEED_L);
  analogWrite(MOTOR_L_1, 0);
}
static inline void Motors_turnRight() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, MOTOR_DEFAULT_SPEED_R);
  analogWrite(MOTOR_L_2, 0);
  analogWrite(MOTOR_L_1, MOTOR_DEFAULT_SPEED_L);
}
// Right encoder interrupt handler
void rightEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
  encoderA_counter += (stateA == stateB) ? 1.0075 : -1;
}
// Left encoder interrupt handler
void leftEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
  encoderB_counter += (stateA == stateB) ? 1 : -1;
}
void initGyroRate() {
  initStartTime = millis();
  previousTime = micros();
  totalYawIncrease = 0;

  while (millis() - initStartTime < initDuration) {
    IMU::read();
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;

    totalYawIncrease += gyroZ * dt;
  }

  double elapsedTime = (millis() - initStartTime) / 1000.0;
  rateOfIncrease = totalYawIncrease / elapsedTime;
  initialized = true;
}
// void updateAngleTask(void *pvParameters) {
//   UNUSED(pvParameters);
//   previousTime = micros();

//   for (;;) {
//     IMU::read();

//     long currentTime = micros();
//     double dt = (currentTime - previousTime) / 1000000.0;
//     previousTime = currentTime;

//     double gyroZ = IMU::getRawGyroZ() / 131.0;
//     currentYaw += (gyroZ - rateOfIncrease) * dt;
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }
/* ------------------------------------------------------------------------------------- */
bool wallFront() {
  return (VLX_readDistance(MIDDLE_VLX) < 5);
}
bool wallRight() {
  return (VLX_readDistance(RIGHT_VLX) < 9);
}
bool wallLeft() {
  return (VLX_readDistance(LEFT_VLX) < 9);
}
void moveForward() {
  Serial.println("Enter MoveForward..");
  target_distance += CELL_WIDTH;
  // Calculate the current distance traveled
  float current_distance = 0;
  // Check if the car has reached the target distance
  while (current_distance < target_distance) {
    // Serial.println(wallRight());
    // Serial.println(wallLeft());
    // Serial.println(wallFront());
    // Calculate the PID error based on target distance and current distance
    error = target_distance - current_distance;
    // Compute PID output for motor speed
    control_output = Kp * error;
    control_output = constrain(control_output, MIN_SPEED, MAX_SPEED);
    // Synchronize the motors based on encoder values and apply the PID-controlled speed
    synchronizeMotors(control_output);
    current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);
  }
  stopMotors();
  Serial.println("Exit MoveForward..");
}
void turnRight() {
  Serial.println("Enter turnRight..");
  desiredAngle = -90;
  if (!initialized) {
    return;
  }
  currentYaw = 0;

  previousTime = micros();
  while (abs(currentYaw - desiredAngle) > 0.3) {

    IMU::read();

    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;
    currentYaw += (gyroZ - rateOfIncrease) * dt;

    Serial.println(currentYaw);
    if (currentYaw < desiredAngle - 0.3) {
      Motors_turnLeft();
    } else if (currentYaw > desiredAngle + 0.3) {
      Motors_turnRight();
    }
  }  // DELETE THIS ==============================================
  stopMotors();
  encoderA_counter = 0;
  encoderB_counter = 0;
  target_distance = 0;
  Serial.println("Exit turnRight..");
}
void turnLeft() {
  Serial.println("Enter turnLeft..");
  desiredAngle = 90;
  if (!initialized) {
    return;
  }
  currentYaw = 0;
  previousTime = micros();
  while (abs(currentYaw - desiredAngle) > 0.3) {
    IMU::read();

    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;
    currentYaw += (gyroZ - rateOfIncrease) * dt;

    if (currentYaw < desiredAngle - 0.3) {
      Motors_turnLeft();
    } else if (currentYaw > desiredAngle + 0.3) {
      Motors_turnRight();
    }
  }
  stopMotors();
  encoderA_counter = 0;
  encoderB_counter = 0;
  target_distance = 0;
  Serial.println("Exit turnLeft..");
}
int orientation(int orient, char turning)

{

  if (turning == 'L') {

    orient -= 1;

    if (orient == -1)

      orient = 3;

  } else if (turning == 'R') {

    orient += 1;

    if (orient == 4)

      orient = 0;

  } else if (turning == 'B') {

    if (orient == 0)

      orient = 2;

    else if (orient == 1)

      orient = 3;

    else if (orient == 2)

      orient = 0;

    else if (orient == 3)

      orient = 1;
  }

  return orient;
}



void updateCoordinates(int orient, int *new_x, int *new_y)

{

  if (orient == 0)
    *new_y += 1;
  else if (orient == 1)
    *new_x += 1;
  else if (orient == 2)
    *new_y -= 1;
  else if (orient == 3)
    *new_x -= 1;
}
