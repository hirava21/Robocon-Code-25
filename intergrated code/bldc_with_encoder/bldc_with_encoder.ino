#include <Servo.h>

// Motor control
Servo myBLDC;

// Encoder variables
volatile unsigned int counter = 0;  // Counter for encoder pulses
unsigned long lastTime = 0;         // To store the last time value was sent
const long interval = 1000;         // Interval to calculate RPM (1000 ms = 1 second)
const int pulsesPerRevolution = 600;  // Encoder's pulses per revolution (PPR)

// Target RPM
const int targetRPM = 500;

void setup() {
  // Serial for debugging
  Serial.begin(9600);

  // Motor control setup
  myBLDC.attach(9);
  StopBLDC();  // Ensure motor is stopped initially

  // Encoder setup
  pinMode(2, INPUT);           // Set pin to input
  digitalWrite(2, HIGH);       // Enable pull-up resistor
  attachInterrupt(0, ai0, RISING);  // Interrupt on digital pin 2, triggered on rising edge
}

void loop() {
  // Check if it's time to calculate and compare RPM
  if (timeInterval()) {
    float rpm = calculateRPM();

    Serial.print("Current RPM = ");
    Serial.println(rpm);

    // Check if target RPM is reached
    if (rpm >= targetRPM) {
      StopBLDC();
      Serial.println("Target RPM reached. Motor stopped.");
    } else {
      // Drive motor to maintain a set speed (adjust value for your motor)
      DriveBLDC(90);  // Example value, may need tuning
    }

    resetSampling();
  }
}

void ai0() {
  // Increment the counter for each pulse
  counter++;
}

float calculateRPM() {
  // Calculate RPM based on encoder pulses
  return (float(counter) * 60) / pulsesPerRevolution;
}

void resetSampling() {
  // Reset counter for the next interval
  counter = 0;
}

boolean timeInterval() {
  unsigned long currentTime = millis();

  // Check if the interval has elapsed
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    return true;
  }
  return false;
}

// BLDC Motor Control Functions
void StopBLDC() {
  myBLDC.write(0);  // Stop motor
}

void DriveBLDC(int val1) {
  myBLDC.write(val1);  // Drive motor with specified speed
}
