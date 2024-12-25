// Define the hardware serial port for TF Luna
#define TFLUNA_SERIAL Serial3

// Variables to hold the distance and strength data
int distance = 0;
int strength = 0;
boolean receiveComplete = false;

void setup() {
  // Start the serial communication for the Arduino and TF Luna
  Serial.begin(115200);
  TFLUNA_SERIAL.begin(115200);
  
  Serial.println("TF Luna LiDAR sensor data reading");
}

void loop() {
  // Get TF Luna data
  getTFminiData(&distance, &strength, &receiveComplete);
  
  // If data is received completely, print the distance and strength
  if (receiveComplete) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Strength: ");
    Serial.println(strength);
    
    receiveComplete = false; // Reset the flag
  }
  
  delay(100); // Small delay for stability
}

void getTFminiData(int *distance, int *strength, boolean *complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  while (TFLUNA_SERIAL.available()) {
    rx[i] = TFLUNA_SERIAL.read();
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
