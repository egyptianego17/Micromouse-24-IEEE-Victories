// Define motor and encoder pins
const int MOTOR_R_1 = 9;
const int MOTOR_R_2 = 10;
const int MOTOR_L_1 = 11;
const int MOTOR_L_2 = 12;

const int MOTOR_R_ENC_A = 2; // Right encoder pin A
const int MOTOR_R_ENC_B = 3; // Right encoder pin B
const int MOTOR_L_ENC_A = 4; // Left encoder pin A
const int MOTOR_L_ENC_B = 5; // Left encoder pin B

// Wheel circumference in cm
const float wheelCircumference = 3.3; // cm
const float distance_per_tick = wheelCircumference / 100.0; // cm per encoder tick

// Encoder counters
volatile uint32_t encoderA_counter = 0; // Right encoder
volatile uint32_t encoderB_counter = 0; // Left encoder

// Time variables
unsigned long startTime;
const unsigned long duration = 10000;  // Duration to move forward in milliseconds

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
            delay(100); // Short delay to avoid multiple prints per second
        }
    } else {
        stop();  // Stop the robot after the duration
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
