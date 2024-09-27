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
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.5;  // Derivative gain

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
const int MIN_SPEED = 80;  // Minimum motor speed (PWM)
const int MAX_SPEED = 150;  // Maximum motor speed (PWM)

void setup() {
  Serial.begin(115200);
  delay(2000);
  motorInit();
}

void loop() {
  // Call the PID function to handle movement
  PID(20);  // Move 20 cm forward
}

// Function to encapsulate PID control and movement logic
void PID(int target_distance) {
  // Calculate the current distance traveled
  float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

  // Calculate error between target and current distance
  error = target_distance - current_distance;
  
  // PID calculations
  integral += error;  // Integral term accumulates the error over time
  derivative = error - previous_error;  // Derivative term tracks the rate of change of error
  control_output = Kp * error + Ki * integral + Kd * derivative;  // PID control equation
  
  // Constrain control_output within motor speed limits
  control_output = constrain(control_output, MIN_SPEED, MAX_SPEED);

  // Apply the PID control output as the motor speed
  moveForward(control_output);

  // Store the current error as previous error for next iteration
  previous_error = error;

  // Print current distance and motor speed for debugging
  Serial.print("Distance: ");
  Serial.print(current_distance);
  Serial.print(" cm | Motor Speed: ");
  Serial.println(control_output);

  delay(50);  // Small delay for loop stability

  // Check if the car has reached the target distance (20 cm)
  if (abs(error) < 0.1) {  // Stop when the error is very small (close to target)
    stop();  // Stop the motors
    Serial.println("Movement complete.");
    while (true);  // Stop execution
  }
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

// Function to move forward with a given speed
void moveForward(int speed) {
  analogWrite(MOTOR_R_1, speed);
  digitalWrite(MOTOR_R_2, LOW);
  analogWrite(MOTOR_L_1, speed - 10);  // Adjust for any imbalance
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to stop the motors
void stop() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, 0);
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