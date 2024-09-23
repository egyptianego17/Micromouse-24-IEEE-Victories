#include "KalmanMPU6050.h"
#include <STM32FreeRTOS.h>

// Motor pins
#define MOTOR_R_1 PB0
#define MOTOR_R_2 PB14
#define MOTOR_L_1 PB9
#define MOTOR_L_2 PB8
#define ROTATE_ANGLE 90
const int MOTOR_R_ENC_A = PA2;
const int MOTOR_R_ENC_B = PA3; 
const int MOTOR_L_ENC_A = PA4; 
const int MOTOR_L_ENC_B = PA5; 
volatile float currentYaw = 0.0; // Global variable to store the yaw angle

// Task handle for the angle updating task
TaskHandle_t updateAngleTaskHandle;

// Wheel circumference in cm
const float wheelCircumference = 10.362; // cm
const float distance_per_tick = wheelCircumference; // cm per encoder tick

// Encoder counters
volatile uint32_t encoderA_counter = 0; // Right encoder
volatile uint32_t encoderB_counter = 0; // Left encoder

// Global variables
double targetYaw = 90.0;   
double yaw = 0;           
double rateOfIncrease = 0; 
bool initialized = false;
double totalYawIncrease = 0;
double desiredAngle = 0;

// Time variables
long previousTime = 0;
long initStartTime = 0;
long initDuration = 10000;

// PID constants
float Kp = 2.0;  
float Ki = 0.1; 
float Kd = 0.5;  

float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float control_output = 0.0;

void initGyroRate();
void turnRight();
void turnLeft();
void stopMotors();
void turnRightM();
void turnLeftM();
void moveForward(void);
void stopMotors(void);
float calculateTotalDistance();
float calculate_distance(uint32_t pulse_count);
void updateAngleTask(void* pvParameters);
void rightEncoderCb();
void leftEncoderCb();
void left(void);

void setup() {
  Serial.begin(115200);

  IMU::init();
  IMU::read();

  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_ENC_A, INPUT);
  pinMode(MOTOR_R_ENC_B, INPUT);
  pinMode(MOTOR_L_ENC_A, INPUT);
  pinMode(MOTOR_L_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_B), rightEncoderCb, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_B), leftEncoderCb, CHANGE);
  initGyroRate();
  xTaskCreate(updateAngleTask,
    "Task1",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    &updateAngleTaskHandle);
  vTaskStartScheduler();

  Serial.println(F("Die"));
}

void loop() {
  left();
  Serial.println(abs(currentYaw - desiredAngle));
  delay(2000);
}

void updateAngleTask(void *pvParameters) {
  UNUSED(pvParameters);
  previousTime = micros();

  for (;;) {
    IMU::read();
    
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0; 
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;
    currentYaw += (gyroZ - rateOfIncrease) * dt;
    // Ensure the yaw is in the 360-degree range (optional, depending on your use case)
    // if (currentYaw > 360.0) {
    //   currentYaw -= 360.0;
    // } else if (currentYaw < 0.0) {
    //   currentYaw += 360.0;
    // }
    // Sleep for 50 milliseconds.
    vTaskDelay((50 * configTICK_RATE_HZ) / 1000L);
  }
}

void left(void) {
  desiredAngle += ROTATE_ANGLE;
  turnLeft();
}

void moveForwardM() {
  digitalWrite(MOTOR_R_1, 1);
  digitalWrite(MOTOR_R_2, 0);
  digitalWrite(MOTOR_L_1, 1);
  digitalWrite(MOTOR_L_2, 0);
}

void turnRight() {
  if (!initialized) {
    Serial.println("Gyroscope not initialized.");
    return;
  }

  yaw = 0;
  previousTime = micros();

  while (abs(yaw)-90.0 < 1) {
    IMU::read();
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;
    yaw += (gyroZ - rateOfIncrease) * dt;

    turnRightM();
    Serial.print("Turning Right. Current Yaw: ");
    Serial.println(yaw);
  }

  stopMotors();
  Serial.println("Completed 90-degree right turn.");
}

void turnLeft() {
  if (!initialized) {
    Serial.println("Gyroscope not initialized.");
    return;
  }

  previousTime = micros();
  while (abs(currentYaw - desiredAngle) > 0.3) {
    Serial.println("abs(currentYaw - desiredAngle)");
    Serial.println(abs(currentYaw - desiredAngle)); 
    Serial.println("yaw < desiredAngle - 0.3");
    Serial.println(currentYaw < desiredAngle);
    if (currentYaw < desiredAngle - 0.3) {
      Serial.print("Left");
      turnLeftM();
    }
    else if (currentYaw > desiredAngle + 0.3) {
      Serial.print("Right");
      turnRightM();
    }
  }

  stopMotors();
  Serial.println("Completed 90-degree turn.");
}

void rightEncoderCb() {
    uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
    encoderA_counter += (stateA == stateB) ? 1 : -1;
}

void leftEncoderCb() {
    uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
    encoderB_counter += (stateA == stateB) ? 1 : -1;
}


float calculate_distance(uint32_t pulse_count) {
  float revolutions = (float)(pulse_count) / 200; 
  return revolutions * wheelCircumference; 
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

  Serial.println("Gyroscope initialization complete.");
  Serial.print("Rate of increase: ");
  Serial.println(rateOfIncrease);
}

void moveForward(void) {
  int target_distance = 17;
  encoderA_counter = 0;
  encoderB_counter = 0;
  int current_distance = 0;
  while (current_distance < target_distance) {
    float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

    moveForwardM();

    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm | Motor Speed: ");
  }
}

void stopMotors() {
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
}

void turnLeftM()
{
  digitalWrite(MOTOR_R_1, 1);
  digitalWrite(MOTOR_R_2, 0);
  digitalWrite(MOTOR_L_2, 1);
  digitalWrite(MOTOR_L_1, 0);
}


void turnRightM()
{
  digitalWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, 1);
  digitalWrite(MOTOR_L_2, 0);
  digitalWrite(MOTOR_L_1, 1);
}

float calculateTotalDistance() {
    float encoder_ticks = (encoderA_counter + encoderB_counter) / 2.0;
    return encoder_ticks * distance_per_tick;
}