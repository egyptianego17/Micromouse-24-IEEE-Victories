#include <Adafruit_VL6180X.h>


// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32



// set the pins to shutdown
#define SHT_LOX1 7  //Right
#define SHT_LOX2 6  //Middle
#define SHT_LOX3 8  //Left

#define COUNT_SENSORS 3

#define RIGHT_VLX 0
#define Middle_VLX 1
#define LEFT_VLX 2

String sensorsNames[3] = { "Right_Sensor", "Middle_Sensor", "Left_Sensor" };

// // Optional define GPIO pins to check to see if complete
// #define GPIO_LOX1 4
// #define GPIO_LOX2 3


// #define TIMING_PIN 13

// objects for the VL6180X
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();


// // Setup mode for doing reads
// typedef enum {RUN_MODE_DEFAULT, RUN_MODE_TIMED, RUN_MODE_ASYNC, RUN_MODE_GPIO, RUN_MODE_CONT} runmode_t;

// runmode_t run_mode = RUN_MODE_DEFAULT;
// uint8_t show_command_list = 1;

//==========================================================================
// Define some globals used in the continuous range mode
// Note: going to start table drive this part, may back up and do the rest later
// Adafruit_VL6180X *sensors[] = {&lox1, &lox2, &lox3};
// const uint8_t COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
// const int sensor_gpios[COUNT_SENSORS] = {GPIO_LOX1, GPIO_LOX2}; // if any are < 0 will poll instead
uint8_t tempRange;
uint8_t sensor_ranges[COUNT_SENSORS];
uint8_t sensor_status[COUNT_SENSORS];
// Could do with uint8_t for 8 sensors, but just in case...
// const uint16_t  ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
// uint16_t        sensors_pending = ALL_SENSORS_PENDING;
// uint32_t        sensor_last_cycle_time;


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
*/
void setID() {
  Serial.println("Starting...");
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // initing LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot Right VL6180X"));
    while (1)
      ;
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot Middle VL6180X"));
    while (1)
      ;
  }
  lox2.setAddress(LOX2_ADDRESS);
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot Left VL6180X"));
    while (1)
      ;
  }
  lox3.setAddress(LOX3_ADDRESS);
  delay(10);
}

void readSensor(Adafruit_VL6180X &vl) {

  // float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  uint8_t range = vl.readRange();

  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    tempRange = range;  //save it for the moment
  }

  // Some error occurred, print it out!

  if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.print("(System error)");
  } else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.print("(ECE failure)");
  } else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.print("(No convergence)");
  } else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.print("(Ignoring range)");
  } else if (status == VL6180X_ERROR_SNR) {
    Serial.print("Signal/Noise error");
  } else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.print("Raw reading underflow");
  } else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.print("Raw reading overflow");
  } else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.print("Range reading underflow");
  } else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.print("Range reading overflow");
  }
}

void read_sensors() {
  readSensor(lox1);
  sensor_ranges[RIGHT_VLX] = tempRange;  //save it now

  readSensor(lox2);
  sensor_ranges[Middle_VLX] = tempRange;  //save it now

  readSensor(lox3);
  sensor_ranges[LEFT_VLX] = tempRange;  //save it now
  // Serial.println();
}





//===============================================================
// Setup
//===============================================================
void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println("Shutdown pins inited...");

  //   // Enable timing pin so easy to see when pass starts and ends
  //   pinMode(TIMING_PIN, OUTPUT);

  // #ifdef GPIO_LOX1
  //   // If we defined GPIO pins, enable them as PULL UP
  //   pinMode(GPIO_LOX1, INPUT_PULLUP);
  //   pinMode(GPIO_LOX2, INPUT_PULLUP);

  // #endif


  // digitalWrite(TIMING_PIN, LOW);
  // Serial.println("All in reset mode...(pins are low)");

  setID();
}

//===============================================================
// Loop
//===============================================================
void loop() {
  read_sensors();
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(sensorsNames[i]);
    Serial.print(": ");
    Serial.print((sensor_ranges[i] / 10));
    Serial.println("cm");
    Serial.println("-----------------------------");
  }
  delay(100);
}