#include <Wire.h>
#include <MPU6050_light.h>

// Direction: left >> 0 , right >> 1
int direction = 0;

// Motor pins
const int MOTOR_R_1 = 9;
const int MOTOR_R_2 = 10;
const int MOTOR_L_1 = 11;
const int MOTOR_L_2 = 12;

// MPU6050 setup
MPU6050 mpu(Wire);
const float Kp_turn = 1.0;
const float Ki_turn = 0.0;
const float Kd_turn = 0.0;
const float target_angle = 90.0; // Target angle for right turn

// PID variables for turning
float integral_turn = 0.0;
float previous_error_turn = 0.0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.begin();
    mpu.calcOffsets(); // Gyro and accelerometer offsets
    motorInit();
    moveForward();  // Start moving forward (to ensure the robot is moving)
}

void loop() {
    // Call the PID control function for turning
    computePID();
    
    delay(100); // Short delay to avoid flooding the Serial Monitor
}

// Initialize motor pins
void motorInit(void) {
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
}

// Move motors to turn right
void adjustMotorsForTurn(float PID_turn) {

    int left_motor_speed = PID_turn; // 0
    int right_motor_speed = -PID_turn; // 0
    
    if(direction == 1){ // right turn
    // Set motor speeds
    analogWrite(MOTOR_R_1, map(left_motor_speed, -90, 90,120,180)); // >> 150
    analogWrite(MOTOR_R_2, 0);
    analogWrite(MOTOR_L_1, 0);
    analogWrite(MOTOR_L_2, map(right_motor_speed, -90, 90,120,180)); //150
    }
    else if(direction == 0){ // left turn
    // Set motor speeds
    analogWrite(MOTOR_R_1, 0);
    analogWrite(MOTOR_R_2, map(left_motor_speed, -90, 90,120,180));
    analogWrite(MOTOR_L_1, map(right_motor_speed, -90, 90,120,180));
    analogWrite(MOTOR_L_2, 0);
    }
}

void stop() {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, LOW);
}

// Function to compute PID control and handle the turn
void computePID() {
    // Measure the current angle from the MPU6050
    mpu.update();
    float current_angle = mpu.getAngleZ();
    
    // Calculate error for PID control
    float angle_error = target_angle - abs(current_angle);

    // PID control for turning
    integral_turn += angle_error;
    float PID_turn = Kp_turn * angle_error +
                     Ki_turn * integral_turn +
                     Kd_turn * (angle_error - previous_error_turn);
    previous_error_turn = angle_error;

    // if (direction == 0){
    //     PID_turn = -PID_turn; // turning left
        
    // }
    // else if(direction == 1){
    //     PID_turn = PID_turn; // turning right
    // }
    
    // Adjust motor speeds based on PID correction for turning
    if(PID_turn == 0){ while(true); } // to be discussed with api
    adjustMotorsForTurn(PID_turn);

    // Print current angle and PID correction
    Serial.print("Current Angle (degrees): ");
    Serial.println(current_angle);
    Serial.print("PID Turn Value: ");
    Serial.println(PID_turn);

    // Check if the target angle is reached within the margin of error
    if (abs(target_angle - current_angle) < 2.0) { // Allowable error margin (1 degree)
        stop();  // Stop the motors when the target angle is reached
        Serial.println("Turn complete.");
        while (true);  // Stop execution
    }
}