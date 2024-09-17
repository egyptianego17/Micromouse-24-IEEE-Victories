// PID parameters for distance control
float Kp_forward = 1.0;
float Ki_forward = 0.0;
float Kd_forward = 0.0;
float target_distance = 18.0;    // Target distance to move forward (one cell = 18 cm)
float distance_per_tick = 0.05;  // Example: distance the robot moves per encoder tick

// PID parameters for positioning control
float Kp_positioning = 1.0;
float Ki_positioning = 0.0;
float Kd_positioning = 0.0;
float setpoint_distance = 3.0;  // Desired distance from the walls in cm

// Encoder counters
volatile uint32_t left_encoder_ticks = 0;  // Track encoder ticks
volatile uint32_t right_encoder_ticks = 0;
float total_distance = 0.0;  // Total distance traveled

// PID parameters for turning
float Kp_turn = 1.0;
float Ki_turn = 0.0;
float Kd_turn = 0.0;

// Variables to track integral and previous error for forward movement
float integral_forward = 0.0;
float previous_error_forward = 0.0;

// Variables to track integral and previous error for positioning
float integral_positioning = 0.0;
float previous_error_positioning = 0.0;

// Variables to track integral and previous error for turning
float integral_turn = 0.0;
float previous_error_turn = 0.0;

// Function to measure distance from walls (mock function, replace with actual sensor readings)
float measureDistanceFromWalls() {
    // Example: Replace with actual VLX sensor readings
    float left_distance = analogRead(A0);  // Read left sensor
    float right_distance = analogRead(A1); // Read right sensor
    
    // Average distance from both walls
    return (left_distance + right_distance) / 2.0;
}

// Function to measure current angle (mock function, replace with actual MPU readings)
float measureCurrentAngle() {
    // Example: Replace with actual MPU readings
    return analogRead(A2);  // Replace with actual MPU angle reading
}

// Function to update encoder ticks (mock function, replace with actual encoder readings)
void updateEncoders() {
    // Use the encoder ticks from the interrupts
    total_distance = ((left_encoder_ticks + right_encoder_ticks) / 2.0) * distance_per_tick;
}

// Function for PID control of forward movement for one cell (18 cm)
String PID_Forward() {
    // Update total distance traveled
    updateEncoders();
    
    // Distance control
    float distance_error = target_distance - total_distance;
    
    // Update integral term for forward movement
    integral_forward += distance_error;
    
    // Calculate PID correction for forward movement
    float PID_forward = Kp_forward * distance_error + 
                        Ki_forward * integral_forward + 
                        Kd_forward * (distance_error - previous_error_forward);
    
    // Update previous error for forward movement
    previous_error_forward = distance_error;
    
    // Positioning control
    float actual_distance_from_walls = measureDistanceFromWalls();
    float positioning_error = setpoint_distance - actual_distance_from_walls;
    
    // Update integral term for positioning
    integral_positioning += positioning_error;
    
    // Calculate PID correction for positioning
    float PID_positioning = Kp_positioning * positioning_error + 
                            Ki_positioning * integral_positioning + 
                            Kd_positioning * (positioning_error - previous_error_positioning);
    
    // Update previous error for positioning
    previous_error_positioning = positioning_error;
    
    // Adjust motor speeds based on both PID corrections
    adjustMotors(PID_forward, PID_positioning);
    
    // Check if the target distance is reached
    if (total_distance >= target_distance) {
        // Stop the motors when the target distance is reached
        stop();
        return "PID_Forward_Done";
    }
    
    return "PID_Forward";
}

// Function for PID control of turning (left and right)
String PID_Turn(String direction) {
    float desired_angle = 90.0;  // Assuming 90-degree turn
    float current_angle = measureCurrentAngle();
    
    // Calculate error
    float error = desired_angle - current_angle;
    
    // Update integral term
    integral_turn += error;
    
    // Calculate PID correction
    float PID_correction = Kp_turn * error + Ki_turn * integral_turn + Kd_turn * (error - previous_error_turn);
    
    // Update previous error
    previous_error_turn = error;
    
    // Adjust motor speeds based on PID correction
    if (direction == "left") {
        adjustMotorsForTurn(PID_correction, "left");
        return "PID_Turn_Left";
    } else {
        adjustMotorsForTurn(PID_correction, "right");
        return "PID_Turn_Right";
    }
}

// Function to adjust motors based on PID correction for forward movement
void adjustMotors(float PID_forward, float PID_positioning) {
    int base_speed = 150;  // Base speed for both motors
    int left_motor_speed = base_speed + PID_positioning - PID_forward;
    int right_motor_speed = base_speed - PID_positioning - PID_forward;
    
    // Set motor speeds (assuming motor control pins)
    analogWrite(MOTOR_R_1, constrain(left_motor_speed, 0, 255));  // Left motor
    analogWrite(MOTOR_R_2, constrain(right_motor_speed, 0, 255)); // Right motor
}

// Function to adjust motors for turning
void adjustMotorsForTurn(float PID_correction, String direction) {
    int turn_speed = 100;  // Speed for turning
    if (direction == "left") {
        analogWrite(MOTOR_R_1, turn_speed + PID_correction);  // Left motor
        analogWrite(MOTOR_R_2, turn_speed - PID_correction); // Right motor
    } else {
        analogWrite(MOTOR_R_1, turn_speed - PID_correction);  // Left motor
        analogWrite(MOTOR_R_2, turn_speed + PID_correction); // Right motor
    }
}

// Motor and Encoder control
void motorInit(void) {
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_B), rightEncoderCb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_B), leftEncoderCb, CHANGE);
}

void moveForward() {
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
}

void stop() {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, LOW);
}

void moveBackward() {
    digitalWrite(MOTOR_R_2, HIGH);
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    digitalWrite(MOTOR_L_1, LOW);
}

void rightEncoderCb(void) {
    uint8_t stateA = digitalRead(MOTOR_R_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_R_ENC_B);
    stateA == stateB ? right_encoder_ticks++ : right_encoder_ticks--;
}

void leftEncoderCb(void) {
    uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
    stateA == stateB ? left_encoder_ticks++ : left_encoder_ticks--;
}

void setup() {
    motorInit();
    
    // Set up sensor pins
    pinMode(A0, INPUT);  // Left VLX sensor
    pinMode(A1, INPUT);  // Right VLX sensor
    pinMode(A2, INPUT);  // MPU sensor
    
    Serial.begin(9600);  // Initialize serial communication
}

void loop() {
    String action = getAction();  // Assume this function provides the next action
    
    if (action == "Move Forward") {
        String result = PID_Forward();
        Serial.println(result);
    } else if (action == "Turn Left") {
        String result = PID_Turn("left");
        Serial.println(result);
    } else if (action == "Turn Right") {
        String result = PID_Turn("right");
        Serial.println(result);
    }
}

// Mock function to simulate getting actions from the robot's decision-making process
String getAction() {
    // Replace this with your actual logic to get the next action
    return "Move Forward";  // Example action
}
