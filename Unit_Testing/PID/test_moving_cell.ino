// PID parameters and other configurations
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

// Time variables
unsigned long startTime;
const unsigned long duration = 10000;  // Duration to move forward in milliseconds

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

float calculateTotalDistance() {
    float encoder_ticks = (encoderA_counter + encoderB_counter) / 2.0;
    return encoder_ticks * distance_per_tick;
}

void setup() {
    Serial.begin(9600);
    motorInit();
    
    startTime = millis();  // Initialize the start time
    moveForward();         // Start moving forward
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    
    if (elapsedTime < duration) {
        // Print the encoder distance every second
        if (elapsedTime % 1000 == 0) {
            float total_distance = calculateTotalDistance();
            Serial.print("Encoder Distance (cm): ");
            Serial.println(total_distance);
        }
    } else {
        stop();  // Stop the robot after the duration
        Serial.println("Movement complete.");
        while (true);  // Stop execution
    }
}
