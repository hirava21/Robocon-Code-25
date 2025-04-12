// Define hardware serial ports for two TF Luna sensors
#define TFLUNA_SERIAL1 Serial3
#define TFLUNA_SERIAL2 Serial2

// Variables to hold the data for both sensors
int distance1 = 0, strength1 = 0;
int distance2 = 0, strength2 = 0;
boolean receiveComplete1 = false, receiveComplete2 = false;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Start serial communication for both sensors
  TFLUNA_SERIAL1.begin(115200); // First TF Luna
  TFLUNA_SERIAL2.begin(115200); // Second TF Luna

  Serial.println("TF Luna Dual Sensor Setup");
}

void loop() {
  // Read data from the first sensor
  getTFminiData1(TFLUNA_SERIAL1, &distance1, &strength1, &receiveComplete1);
  if (receiveComplete1) {
    Serial.print("Sensor 1 - Distance: ");
    Serial.print(distance1);
    Serial.print(" cm, Strength: ");
    Serial.println(strength1);
    receiveComplete1 = false; // Reset the flag
  }

  // Read data from the second sensor
  getTFminiData2(TFLUNA_SERIAL2, &distance2, &strength2, &receiveComplete2);
  if (receiveComplete2) {
    Serial.print("Sensor 2 - Distance: ");
    Serial.print(distance2);
    Serial.print(" cm, Strength: ");
    Serial.println(strength2);
    receiveComplete2 = false; // Reset the flag
  }

  delay(100); // Small delay for stability
}

// Function to get data from a specified serial port
void getTFminiData1(HardwareSerial &serialPort, int *distance, int *strength, boolean *complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  while (serialPort.available()) {
    rx[i] = serialPort.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
void getTFminiData2(HardwareSerial &serialPort, int *distance, int *strength, boolean *complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  while (serialPort.available()) {
    rx[i] = serialPort.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
