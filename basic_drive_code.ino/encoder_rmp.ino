#include <Encoder.h>

// Define encoder pins
Encoder myEnc(5, 6);

// Encoder specifications
const int PPR = 1000; // Replace with the actual Pulses Per Revolution of your encoder

// Variables for timing and pulse counting
unsigned long lastTime = 0;
unsigned long lastPosition = 0;
float rpm = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("RPM Measurement:");
}

void loop() {
  // Current time
  unsigned long currentTime = millis();

  // Read encoder position
  long currentPosition = myEnc.read();

  // Calculate elapsed time (in milliseconds)
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime >= 1000) { // Update every 1 second
    // Calculate the number of pulses in the elapsed time
    long pulses = currentPosition - lastPosition;

    // Calculate RPM
    rpm = (pulses / (float)PPR) * (60000.0 / elapsedTime);

    // Update for the next calculation
    lastPosition = currentPosition;
    lastTime = currentTime;

    // Print RPM to the Serial Monitor
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
}
