// PID parameters for distance control
float Kp_forward = 1.0;
float Ki_forward = 0.0;
float Kd_forward = 0.0;
float target_distance = 18.0;    // Target distance to move forward (one cell = 18 cm)
float distance_per_tick = 3.3 / 100;  // Circumference of the wheel in cm divided by the number of encoder ticks

// PID parameters for positioning control
float Kp_positioning = 1.0;
float Ki_positioning = 0.0;
float Kd_positioning = 0.0;
float setpoint_distance = 3.0;  // Desired distance from the walls in cm

// PID parameters for turning
float Kp_turn = 1.0;
float Ki_turn = 0.0;
float Kd_turn = 0.0;

// Variables for PID control
float integral_forward = 0.0;
float previous_error_forward = 0.0;
float integral_positioning = 0.0;
float previous_error_positioning = 0.0;
float integral_turn = 0.0;
float previous_error_turn = 0.0;

// Encoder counters
static volatile uint32_t encoderA_counter = 0; // Right encoder
static volatile uint32_t encoderB_counter = 0; // Left encoder

void motorInit(void)
{
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_A), rightEncoderCb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENC_B), rightEncoderCb, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_A), leftEncoderCb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_L_ENC_B), leftEncoderCb, CHANGE);
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
    stateA == stateB ? encoderA_counter++ : encoderA_counter--;
}

void leftEncoderCb(void)
{
    uint8_t stateA = digitalRead(MOTOR_L_ENC_A);
    uint8_t stateB = digitalRead(MOTOR_L_ENC_B);
    stateA == stateB ? encoderB_counter++ : encoderB_counter--;
}

// Function to measure distance from walls (mock function, replace with actual sensor readings)
float measureDistanceFromWalls() {
    float left_distance = analogRead(A0);  // Read left sensor
    float right_distance = analogRead(A1'); // Read right sensor
    
    return (left_distance + right_distance) / 2.0;
}

// Function to measure current angle (mock function, replace with actual MPU readings)
float measureCurrentAngle() {
    return analogRead(A2);  // Replace with actual MPU angle reading
}

// Function to update total distance traveled
float calculateTotalDistance() {
    float encoder_ticks = (encoderA_counter + encoderB_counter) / 2.0;
    return encoder_ticks * distance_per_tick;
}

// PID control for forward movement
String PID_Forward() {
    float total_distance = calculateTotalDistance();
    float distance_error = target_distance - total_distance;
    
    integral_forward += distance_error;
    float PID_forward = Kp_forward * distance_error + 
                        Ki_forward * integral_forward + 
                        Kd_forward * (distance_error - previous_error_forward);
    previous_error_forward = distance_error;
    
    float actual_distance_from_walls = measureDistanceFromWalls();
    float positioning_error = setpoint_distance - actual_distance_from_walls;
    
    integral_positioning += positioning_error;
    float PID_positioning = Kp_positioning * positioning_error + 
                            Ki_positioning * integral_positioning + 
                            Kd_positioning * (positioning_error - previous_error_positioning);
    previous_error_positioning = positioning_error;
    
    adjustMotors(PID_forward, PID_positioning);
    
    if (total_distance >= target_distance) {
        stop();
        return "PID_Forward_Done";
    }
    
    return "PID_Forward";
}

// PID control for turning
String PID_Turn(String direction) {
    float desired_angle = 90.0;  // Assuming 90-degree turn
    float current_angle = measureCurrentAngle();
    
    float error = desired_angle - current_angle;
    integral_turn += error;
    float PID_correction = Kp_turn * error + Ki_turn * integral_turn + Kd_turn * (error - previous_error_turn);
    previous_error_turn = error;
    
    if (direction == "left") {
        adjustMotorsForTurn(PID_correction, "left");
        return "PID_Turn_Left";
    } else {
        adjustMotorsForTurn(PID_correction, "right");
        return "PID_Turn_Right";
    }
}

// Adjust motor speeds based on PID correction for forward movement
void adjustMotors(float PID_forward, float PID_positioning) {
    int base_speed = 150;
    int left_motor_speed = base_speed + PID_positioning - PID_forward;
    int right_motor_speed = base_speed - PID_positioning - PID_forward;
    
    analogWrite(MOTOR_R_1, constrain(left_motor_speed, 0, 255));
    analogWrite(MOTOR_R_2, constrain(right_motor_speed, 0, 255));
}

// Adjust motors for turning
void adjustMotorsForTurn(float PID_correction, String direction) {
    int turn_speed = 100;
    if (direction == "left") {
        analogWrite(MOTOR_R_1, turn_speed + PID_correction);
        analogWrite(MOTOR_R_2, turn_speed - PID_correction);
    } else {
        analogWrite(MOTOR_R_1, turn_speed - PID_correction);
        analogWrite(MOTOR_R_2, turn_speed + PID_correction);
    }
}

void setup() {
    Serial.begin(9600);
    motorInit();
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
