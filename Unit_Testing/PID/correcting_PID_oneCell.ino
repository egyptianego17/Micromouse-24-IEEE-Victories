// Pin definitions for motors and encoders
#define MOTOR_R_1 PB0
#define MOTOR_R_2 PB14
#define MOTOR_L_1 PB9
#define MOTOR_L_2 PB8

#define MOTOR_R_ENC_A PA2
#define MOTOR_R_ENC_B PA3

#define MOTOR_L_ENC_A PA4
#define MOTOR_L_ENC_B PA5

// Wheel and encoder constants
const float wheelCircumference = 10.362; // cm
const float distance_per_tick = wheelCircumference / 200; // cm per encoder tick (assuming 200 ticks per revolution)

// PID constants
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.5;

// PID variables
float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float control_output = 0.0;

// Encoder counters
volatile long int encoderA_counter = 0;  // Right encoder
volatile long int encoderB_counter = 0;  // Left encoder

// Motor speed limits
const int MIN_SPEED = 80;
const int MAX_SPEED = 150;
const int TOLERANCE = 1;  // Step difference tolerance between encoders

void setup() {
  Serial.begin(115200);
  delay(2000);
  motorInit();
}

void loop() {
  // Call the PID function to handle movement with a target distance of 20 cm
  PID(20.0);  
}

// Function to encapsulate PID control and movement logic
void PID(float target_distance) {
  // Calculate the current distance traveled
  float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

  // Check if the car has reached the target distance
  if (current_distance < target_distance) {
    // Calculate the PID error based on target distance and current distance
    error = target_distance - current_distance;
    integral += error;
    derivative = error - previous_error;

    // Compute PID output for motor speed
    control_output = Kp * error + Ki * integral + Kd * derivative;
    control_output = constrain(control_output, MIN_SPEED, MAX_SPEED);  // Ensure the speed stays within limits

    // Synchronize the motors based on encoder values and apply the PID-controlled speed
    synchronizeMotors(control_output);

    previous_error = error;

    // Print current distance and motor speed for debugging
    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm | Motor Speed: ");
    Serial.println(control_output);

    delay(100);  // Small delay for loop stability
  } else {
    // Stop the car once it reaches the target distance
    stop();
    Serial.println("Movement complete.");
    while (true);  // Stop execution
  }
}// Pin definitions for motors and encoders
#define MOTOR_R_1 PB0
#define MOTOR_R_2 PB14
#define MOTOR_L_1 PB9
#define MOTOR_L_2 PB8

#define MOTOR_R_ENC_A PA2
#define MOTOR_R_ENC_B PA3

#define MOTOR_L_ENC_A PA4
#define MOTOR_L_ENC_B PA5

// Wheel and encoder constants
const float wheelCircumference = 10.362; // cm
const float distance_per_tick = wheelCircumference / 200; // cm per encoder tick (assuming 200 ticks per revolution)

// PID constants
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.5;

// PID variables
float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float control_output = 0.0;

// Encoder counters
volatile long int encoderA_counter = 0;  // Right encoder
volatile long int encoderB_counter = 0;  // Left encoder

// Motor speed limits
const int MIN_SPEED = 80;
const int MAX_SPEED = 150;
const int TOLERANCE = 1;  // Step difference tolerance between encoders

void setup() {
  Serial.begin(115200);
  delay(2000);
  motorInit();
}

void loop() {
  // Call the PID function to handle movement with a target distance of 20 cm
  PID(20.0);  
}

// Function to encapsulate PID control and movement logic
void PID(float target_distance) {
  // Calculate the current distance traveled
  float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

  // Check if the car has reached the target distance
  if (current_distance < target_distance) {
    // Calculate the PID error based on target distance and current distance
    error = target_distance - current_distance;
    integral += error;
    derivative = error - previous_error;

    // Compute PID output for motor speed
    control_output = Kp * error + Ki * integral + Kd * derivative;
    control_output = constrain(control_output, MIN_SPEED, MAX_SPEED);  // Ensure the speed stays within limits

    // Synchronize the motors based on encoder values and apply the PID-controlled speed
    synchronizeMotors(control_output);

    previous_error = error;

    // Print current distance and motor speed for debugging
    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm | Motor Speed: ");
    Serial.println(control_output);

    delay(100);  // Small delay for loop stability
  } else {
    // Stop the car once it reaches the target distance
    stop();
    Serial.println("Movement complete.");
    while (true);  // Stop execution
  }
}

// Function to synchronize motor movements based on encoder feedback and apply PID speed
void synchronizeMotors(int base_speed) {
  // If both motors are synchronized or within tolerance, move both motors forward
  if (abs(encoderA_counter - encoderB_counter) <= TOLERANCE) {
    moveForward(base_speed);
  }
  // If the right motor encoder has more steps, stop the right motor and let the left motor catch up
  else if (encoderA_counter > encoderB_counter) {
    stopRightMotor();
    moveLeftMotor(base_speed);
  }
  // If the left motor encoder has more steps, stop the left motor and let the right motor catch up
  else if (encoderB_counter > encoderA_counter) {
    stopLeftMotor();
    moveRightMotor(base_speed);
  }
  
  // Print encoder values for debugging
  Serial.print("Right Encoder: ");
  Serial.print(encoderA_counter);
  Serial.print(" | Left Encoder: ");
  Serial.println(encoderB_counter);
  
  delay(10);  // Small delay for stability
}

// Initialize motor and encoder pins
void motorInit(void) {
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);

  pinMode(MOTOR_R_ENC_A, INPUT);
  pinMode(MOTOR_R_ENC_B, INPUT);
  pinMode(MOTOR_L_ENC_A, INPUT);
  pinMode(MOTOR_L_ENC_B, INPUT);

  // Set up interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, RISING);
}

// Function to move both motors forward at a given speed
void moveForward(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to stop both motors
void stop() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, 0);
}

// Function to stop only the right motor
void stopRightMotor() {
  analogWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, LOW);
}

// Function to stop only the left motor
void stopLeftMotor() {
  analogWrite(MOTOR_L_1, 0);
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to move only the right motor
void moveRightMotor(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
}

// Function to move only the left motor
void moveLeftMotor(int speed) {
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}

// Right encoder interrupt handler
void rightEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
  encoderA_counter += (stateA == stateB) ? 1 : -1;
}

// Left encoder interrupt handler
void leftEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
  encoderB_counter++;
}

// Function to calculate the distance based on encoder pulses
float calculate_distance(uint32_t pulse_count) {
  float revolutions = (float)(pulse_count) / 200;  // Assuming 200 pulses per revolution
  return revolutions * wheelCircumference;  // Distance in cm
}


// Function to synchronize motor movements based on encoder feedback and apply PID speed
void synchronizeMotors(int base_speed) {
  // If both motors are synchronized or within tolerance, move both motors forward
  if (abs(encoderA_counter - encoderB_counter) <= TOLERANCE) {
    moveForward(base_speed);
  }
  // If the right motor encoder has more steps, stop the right motor and let the left motor catch up
  else if (encoderA_counter > encoderB_counter) {
    stopRightMotor();
    moveLeftMotor(base_speed);
  }
  // If the left motor encoder has more steps, stop the left motor and let the right motor catch up
  else if (encoderB_counter > encoderA_counter) {
    stopLeftMotor();
    moveRightMotor(base_speed);
  }
  
  // Print encoder values for debugging
  Serial.print("Right Encoder: ");
  Serial.print(encoderA_counter);
  Serial.print(" | Left Encoder: ");
  Serial.println(encoderB_counter);
  
  delay(10);  // Small delay for stability
}

// Initialize motor and encoder pins
void motorInit(void) {
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);

  pinMode(MOTOR_R_ENC_A, INPUT);
  pinMode(MOTOR_R_ENC_B, INPUT);
  pinMode(MOTOR_L_ENC_A, INPUT);
  pinMode(MOTOR_L_ENC_B, INPUT);

  // Set up interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, RISING);
}

// Function to move both motors forward at a given speed
void moveForward(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to stop both motors
void stop() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, 0);
}

// Function to stop only the right motor
void stopRightMotor() {
  analogWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, LOW);
}

// Function to stop only the left motor
void stopLeftMotor() {
  analogWrite(MOTOR_L_1, 0);
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to move only the right motor
void moveRightMotor(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
}

// Function to move only the left motor
void moveLeftMotor(int speed) {
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}

// Right encoder interrupt handler
void rightEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
  encoderA_counter += (stateA == stateB) ? 1 : -1;
}

// Left encoder interrupt handler
void leftEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
  encoderB_counter++;
}

// Function to calculate the distance based on encoder pulses
float calculate_distance(uint32_t pulse_count) {
  float revolutions = (float)(pulse_count) / 200;  // Assuming 200 pulses per revolution
  return revolutions * wheelCircumference;  // Distance in cm
}
