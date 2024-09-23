#include "KalmanMPU6050.h"
#include <STM32FreeRTOS.h>

// Motor pins
#define MOTOR_R_1 PB0
#define MOTOR_R_2 PB14
#define MOTOR_L_1 PB9
#define MOTOR_L_2 PB8
#define ROTATE_ANGLE 81.8
const int MOTOR_R_ENC_A = PA2; // Right encoder pin A
const int MOTOR_R_ENC_B = PA3; // Right encoder pin B
const int MOTOR_L_ENC_A = PA4; // Left encoder pin A
const int MOTOR_L_ENC_B = PA5; // Left encoder pin B

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
double targetYaw = 90.0;   // Target yaw for the next turn (90-degree increments)
double yaw = 0;            // Current yaw in degrees
double rateOfIncrease = 0; // Rate of increase in yaw during initialization
bool initialized = false;
double totalYawIncrease = 0; // Total yaw increase during initialization
double desiredAngle = 0;
// Time variables
long previousTime = 0;
long initStartTime = 0;
long initDuration = 10000; // 10 seconds in milliseconds

// PID constants
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.5;  // Derivative gain

// PID variables
float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float control_output = 0.0;


// Function prototypes
void initGyroRate();
void turnRight();
void turnLeft();
void stopMotors();
void turnRightM(int speed);
void turnLeftM(int speed);
void moveForward(void);
void stopMotors(void);
float calculateTotalDistance();
float calculate_distance(uint32_t pulse_count);
void updateAngleTask(void* pvParameters);
void setup() {
  Serial.begin(115200);

  // Initialize IMU
  IMU::init();
  IMU::read();

  // Motor pins setup
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
  // Initialize gyroscope rate
  initGyroRate();
  // create print task
  xTaskCreate(updateAngleTask,
    "Task1",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    &updateAngleTaskHandle);
  // start FreeRTOS
  vTaskStartScheduler();

  // should never return
  Serial.println(F("Die"));
  // while(1);
}

void loop() {
  // Example usage: turn 90 degrees to the right
  // turnRight();
  turnLeft();
  abs(currentYaw - desiredAngle);
  delay(2000);
  // Serial.print("Current Yaw: ");
  // Serial.println(currentYaw);

  // moveForward();
  // turnLeft();
  // turnLeft();
  // turnLeft();
  // turnLeft();
  // stopMotors();
  // delay(2000);
  // moveForward();
  // turnLeft();
  // turnLeft();
  // turnLeft();
  // // Pause for a moment before the next action
  // delay(2000);

  // // Example usage: turn 90 degrees to the left
  // turnLeft();

  // // Pause for a moment before the next action
  // delay(2000);
}

void updateAngleTask(void *pvParameters) {
  UNUSED(pvParameters);
  previousTime = micros();

  // Flash led every 200 ms.
  for (;;) {
    // Read IMU data
    IMU::read();
    
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0; // Convert to seconds
    previousTime = currentTime;

    // Get Z-axis gyro data and calculate yaw
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
  desiredAngle += 90;
  turnLeft();
}

void moveForwardM(int speed) {
  analogWrite(MOTOR_R_1, 120);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 120);
  analogWrite(MOTOR_L_2, 0);
}

/**
 * Turn 90 degrees to the right.
 */
void turnRight() {
  if (!initialized) {
    Serial.println("Gyroscope not initialized.");
    return;
  }

  yaw = 0; // Reset yaw to 0 for this turn
  previousTime = micros();

  while (abs(yaw)-90.0 < 1) {
    IMU::read();
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0;
    yaw += (gyroZ - rateOfIncrease) * dt;

    // Set motors to turn right
    turnRightM(120 );
    // Output current yaw for debugging
    Serial.print("Turning Right. Current Yaw: ");
    Serial.println(yaw);
  }

  stopMotors();
  Serial.println("Completed 90-degree right turn.");
}

/**
 * Turn 90 degrees to the left.
 */
void turnLeft() {
  if (!initialized) {
    Serial.println("Gyroscope not initialized.");
    return;
  }

  previousTime = micros();
  while (abs(currentYaw - desiredAngle) > 0.3) {
    Serial.println(abs(currentYaw - desiredAngle)); 
    // Check yaw and adjust motor control accordingly
    if (yaw < desiredAngle - 0.3) {
      turnLeftM(120);
    }
    else if (yaw > desiredAngle + 0.3) {
      turnRightM(120);
    }

    // Output current yaw for debugging
    Serial.print("Adjusting turn. Current Yaw: ");
    Serial.println(yaw);
  }

  // Stop the motors once yaw is within acceptable range
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


// Function to calculate the distance based on encoder pulses
float calculate_distance(uint32_t pulse_count) {
  float revolutions = (float)(pulse_count) / 200;  // Assuming 200 pulses per revolution
  return revolutions * wheelCircumference;  // Distance in cm
}

/**
 * Initialize the gyroscope rate by calculating drift during a fixed time interval.
 */
void initGyroRate() {
  initStartTime = millis();
  previousTime = micros();
  totalYawIncrease = 0;
  
  // Collect data to calculate the rate of yaw increase (gyro drift)
  while (millis() - initStartTime < initDuration) {
    IMU::read();
    long currentTime = micros();
    double dt = (currentTime - previousTime) / 1000000.0; // Time in seconds
    previousTime = currentTime;

    double gyroZ = IMU::getRawGyroZ() / 131.0; // Assuming 131 LSB/°/s

    totalYawIncrease += gyroZ * dt;
  }

  // Calculate rate of increase based on the total yaw drift during the initialization period
  double elapsedTime = (millis() - initStartTime) / 1000.0; // Convert milliseconds to seconds
  rateOfIncrease = totalYawIncrease / elapsedTime;
  initialized = true;

  Serial.println("Gyroscope initialization complete.");
  Serial.print("Rate of increase: ");
  Serial.println(rateOfIncrease);
}

// Function to encapsulate PID control and movement logic
void moveForward(void) {
  int target_distance = 17;
  // Calculate the current distance traveled
  encoderA_counter = 0;
  encoderB_counter = 0;
  int current_distance = 0;
  // Check if the car has reached the target distance (20 cm)
  while (current_distance < target_distance) {
    float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

    // Apply the motor speed to move forward
    moveForwardM(100);

    // Print current distance and motor speed for debugging
    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm | Motor Speed: ");
  }
}

/**
 * Stop both motors.
 */
void stopMotors() {
  analogWrite(MOTOR_R_1, LOW);
  analogWrite(MOTOR_R_2, LOW);
  analogWrite(MOTOR_L_1, LOW);
  analogWrite(MOTOR_L_2, LOW);
  Serial.println("Motors stopped.");
}

void turnLeftM(int speed)
{
  analogWrite(MOTOR_R_1, 0  );
  analogWrite(MOTOR_R_2, speed);
  analogWrite(MOTOR_L_2, speed);
  analogWrite(MOTOR_L_1, 0);
}


void turnRightM(int speed)
{
  analogWrite(MOTOR_R_1, speed);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_2, 0);
  analogWrite(MOTOR_L_1, speed);
}

float calculateTotalDistance() {
    float encoder_ticks = (encoderA_counter + encoderB_counter) / 2.0;
    return encoder_ticks * distance_per_tick;
}

