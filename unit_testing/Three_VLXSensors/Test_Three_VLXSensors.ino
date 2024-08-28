#include <Adafruit_VL6180X.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 PC15  //Right
#define SHT_LOX2 PC14  //Middle
#define SHT_LOX3 8     //Left

#define COUNT_SENSORS 3

#define RIGHT_VLX 0
#define Middle_VLX 1
#define LEFT_VLX 2

String sensorsNames[3] = { "Right_Sensor", "Middle_Sensor", "Left_Sensor" };

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

uint8_t tempRange;
uint8_t sensor_ranges[COUNT_SENSORS];
uint8_t sensor_status[COUNT_SENSORS];

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

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
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