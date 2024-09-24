// Motor and encoder pins
const int MOTOR_R_1 = 9;
const int MOTOR_R_2 = 10;
const int MOTOR_L_1 = 11;
const int MOTOR_L_2 = 12;

const int MOTOR_R_ENC_A = 2; // Right encoder pin A
const int MOTOR_R_ENC_B = 3; // Right encoder pin B
const int MOTOR_L_ENC_A = 4; // Left encoder pin A
const int MOTOR_L_ENC_B = 5; // Left encoder pin B

// Wheel circumference and encoder parameters
const float wheelCircumference = 10.362; // cm
const int encoderTicksPerRevolution = 200; // Number of encoder ticks per wheel revolution
const float distance_per_tick = wheelCircumference / encoderTicksPerRevolution; // cm per encoder tick

// Target distance to move forward (one cell = 18 cm)
const float target_distance = 18.0; 

// PID parameters for forward movement
const float Kp_forward = 1.0;
const float Ki_forward = 0.0;
const float Kd_forward = 0.0;

// PID parameters for positioning
const float Kp_positioning = 1.0;
const float Ki_positioning = 0.0;
const float Kd_positioning = 0.0;

// PID variables
float integral_forward = 0.0;
float previous_error_forward = 0.0;
float integral_positioning = 0.0;
float previous_error_positioning = 0.0;

// Encoder counters
volatile uint32_t encoderA_counter = 0; // Right encoder
volatile uint32_t encoderB_counter = 0; // Left encoder

// Distance and positioning variables
float total_distance = 0.0;
const float setpoint_distance = 3.0; // Desired distance from walls in cm

void setup() {
    Serial.begin(9600);
    motorInit();
    moveForward();  // Start moving forward
}

void loop() {
    PID_move_forward();
}
////////////////////////////////////////////////////////////////////////////////////////
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

float calculateTotalDistance() {
    float encoder_ticks = (encoderA_counter + encoderB_counter) / 2.0;
    return encoder_ticks * distance_per_tick;
}

// Mock function to measure distance from walls (replace with actual sensor readings)
float measureDistanceFromWalls() {
    // Example: Replace with actual VLX sensor readings
    float left_distance = analogRead(A0);  // Read left sensor
    float right_distance = analogRead(A1); // Read right sensor
    
    // Average distance from both walls
    return (left_distance + right_distance) / 2.0;
}

// Function to adjust motors based on PID corrections
void adjustMotors(float PID_forward, float PID_positioning) {
    int base_speed = 150;  // Base speed for both motors
    int left_motor_speed = base_speed + PID_positioning - PID_forward;
    int right_motor_speed = base_speed - PID_positioning - PID_forward;
    
    // Set motor speeds (assuming motor control pins)
    analogWrite(MOTOR_R_1, constrain(left_motor_speed, 0, 255));  // Left motor
    analogWrite(MOTOR_R_2, constrain(right_motor_speed, 0, 255)); // Right motor
}

void PID_move_forward() {
    // Update encoder ticks and distance
    total_distance = calculateTotalDistance();
    float actual_distance_from_walls = measureDistanceFromWalls();

    // Calculate errors for PID control
    float distance_error = target_distance - total_distance;
    float positioning_error = setpoint_distance - actual_distance_from_walls;

    // PID control for forward movement
    integral_forward += distance_error;
    float PID_forward = Kp_forward * distance_error +
                        Ki_forward * integral_forward +
                        Kd_forward * (distance_error - previous_error_forward);
    previous_error_forward = distance_error;

    // PID control for positioning
    integral_positioning += positioning_error;
    float PID_positioning = Kp_positioning * positioning_error +
                            Ki_positioning * integral_positioning +
                            Kd_positioning * (positioning_error - previous_error_positioning);
    previous_error_positioning = positioning_error;

    // Adjust motor speeds based on PID corrections
    adjustMotors(PID_forward, PID_positioning);

    // Print encoder distance and positioning error
    Serial.print("Encoder Distance (cm): ");
    Serial.println(total_distance);
    Serial.print("Positioning Error (cm): ");
    Serial.println(positioning_error);
    delay(100); // Short delay to avoid flooding the Serial Monitor

    // Check if the target distance is reached
    if (total_distance >= target_distance) {
        stop();  // Stop the motors when the target distance is reached
        Serial.println("Movement complete.");
        while (true);  // Stop execution
    }
}
