// // Pin definitions for motors and encoders
// #define MOTOR_R_1 PB0
// #define MOTOR_R_2 PB14
// #define MOTOR_L_1 PB9
// #define MOTOR_L_2 PB8

// #define MOTOR_R_ENC_A PA2
// #define MOTOR_R_ENC_B PA3

// #define MOTOR_L_ENC_A PA4
// #define MOTOR_L_ENC_B PA5

// // Wheel and encoder constants
// const float wheelCircumference = 10.362; // cm
// const float distance_per_tick = wheelCircumference / 200; // cm per encoder tick (assuming 200 ticks per revolution)
// const float target_distance = 20.0; // Target distance to move forward (20 cm)

// // PID constants
// float Kp = 2.0;  // Proportional gain
// float Ki = 0.1;  // Integral gain
// float Kd = 0.5;  // Derivative gain

// // PID variables
// float error = 0.0;
// float previous_error = 0.0;
// float integral = 0.0;
// float derivative = 0.0;
// float control_output = 0.0;

// int i = 0;

// // Encoder counters
// volatile long int encoderA_counter = 0;  // Right encoder
// volatile long int encoderB_counter = 0;  // Left encoder

// void setup() {
//   Serial.begin(115200);
//   motorInit();
// }

// void loop() {
//   // Call the PID function to handle movement
//   PID();
// }

// // Function to encapsulate PID control and movement logic
// void PID() {
//   // Calculate the current distance traveled
//   float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

//   // Check if the car has reached the target distance (20 cm)
//   if (current_distance < target_distance) {
//     // PID calculations
//     error = target_distance - current_distance;  // Calculate the error
//     integral += error;  // Calculate the integral 
//     derivative = error - previous_error;  // Calculate the derivative term

//     // PID control output
//     control_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

//     // Adjust motor speeds based on control output (clamp between 0 and 255 for PWM)
//     int motor_speed = constrain((int)control_output, 0, 255);

//     // Apply the motor speed to move forward
//     moveForward(motor_speed);

//     // Update the previous error
//     previous_error = error;

//     // Print current distance and motor speed for debugging
//     Serial.print("Distance: ");
//     Serial.print(current_distance);
//     Serial.print(" cm | Motor Speed: ");
//     Serial.println(motor_speed);
//     Serial.println(i);

//     delay(100);  // Small delay for loop stability
//   } else {
//     // Stop the car once it reaches the target distance
//     stop();
//     Serial.println("Movement complete.");
//     while (true);  // Stop execution
//   }
// }

// // Initialize motor and encoder pins
// void motorInit(void) {
//   pinMode(MOTOR_R_1, OUTPUT);
//   pinMode(MOTOR_R_2, OUTPUT);
//   pinMode(MOTOR_L_1, OUTPUT);
//   pinMode(MOTOR_L_2, OUTPUT);

//   pinMode(MOTOR_R_ENC_A, INPUT);
//   pinMode(MOTOR_R_ENC_B, INPUT);
//   pinMode(MOTOR_L_ENC_A, INPUT);
//   pinMode(MOTOR_L_ENC_B, INPUT);

//   // Set up interrupts for encoders
//   attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, RISING);
//   attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, RISING);
// }

// // Function to move forward with a given speed
// void moveForward(int speed) {
//   analogWrite(MOTOR_R_1, speed);
//   digitalWrite(MOTOR_R_2, LOW);
//   analogWrite(MOTOR_L_1, speed);
//   digitalWrite(MOTOR_L_2, LOW);
// }

// // Function to stop the motors
// void stop() {
//   analogWrite(MOTOR_R_1, 0);
//   analogWrite(MOTOR_R_2, 0);
//   analogWrite(MOTOR_L_1, 0);
//   analogWrite(MOTOR_L_2, 0);
// }

// // Right encoder interrupt handler.
// void rightEncoderCb() {
//   uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
//   uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
//   encoderA_counter += (stateA == stateB) ? 1 : -1;
//   i++;
// }

// // Left encoder interrupt handler
// void leftEncoderCb() {
//   uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
//   uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
//   encoderB_counter++;
// }

// // Function to calculate the distance based on encoder pulses
// float calculate_distance(uint32_t pulse_count) {
//   float revolutions = (float)(pulse_count) / 200;  // Assuming 200 pulses per revolution
//   return revolutions * wheelCircumference;  // Distance in cm
// }


///////////////////////////////////////////////////////


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
const float target_distance = 20.0; // Target distance to move forward (20 cm)

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

int i = 0;

// Encoder counters
volatile long int encoderA_counter = 0;  // Right encoder
volatile long int encoderB_counter = 0;  // Left encoder

// Motor speed limits
const int MIN_SPEED = 130;  // Minimum motor speed (PWM)
const int MAX_SPEED = 200;  // Maximum motor speed (PWM)

void setup() {
  Serial.begin(115200);
  motorInit();
}

void loop() {
  // Call the PID function to handle movement
  PID();
}

// Function to encapsulate PID control and movement logic
void PID() {
  // Calculate the current distance traveled
  float current_distance = calculate_distance((encoderA_counter + encoderB_counter) / 2);

  // Check if the car has reached the target distance (20 cm)
  if (current_distance < target_distance) {
    // PID calculations
    error = target_distance - current_distance;  // Calculate the error
    integral += error;  // Calculate the integral 
    derivative = error - previous_error;  // Calculate the derivative term

    // PID control output
    control_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Adjust motor speed based on control output
    int motor_speed;
    
    // If the error is small, reduce speed more significantly
    if (error < 5.0) {  // Example threshold for close to target (5 cm)
      motor_speed = map((int)control_output, 0, 255, 0, MIN_SPEED); // Slow down to less than MIN_SPEED
      motor_speed = constrain(motor_speed, 0, MIN_SPEED);
    } else {
      motor_speed = map((int)control_output, 0, 255, MIN_SPEED, MAX_SPEED); // Normal operation between MIN_SPEED and MAX_SPEED
      motor_speed = constrain(motor_speed, MIN_SPEED, MAX_SPEED);
    }

    // Apply the motor speed to move forward
    moveForward(motor_speed);

    // Update the previous error
    previous_error = error;

    // Print current distance and motor speed for debugging
    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm | Motor Speed: ");
    Serial.println(motor_speed);
    Serial.println(i);

    delay(100);  // Small delay for loop stability
  } else {
    // Stop the car once it reaches the target distance
    stop();
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
  analogWrite(MOTOR_L_1, speed);
  digitalWrite(MOTOR_L_2, LOW);
}

// Function to stop the motors
void stop() {
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, 0);
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, 0);
}

// Right encoder interrupt handler.
void rightEncoderCb() {
  uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
  uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
  encoderA_counter += (stateA == stateB) ? 1 : -1;
  i++;
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
