volatile unsigned int counter = 0;  // Counter for encoder pulses

unsigned long lastTime = 0;         // To store the last time value was sent
const long interval = 1000;         // Interval at which to send results (1000 ms = 1 second)

const int pulsesPerRevolution = 1200;  // Encoder's pulses per revolution (PPR)
const float wheelDiameter = 0.25;      // Diameter of the wheel in cm
const float wheelCircumference = 3.14159 * wheelDiameter;  // Circumference of the wheel in cm

float totalDistance = 0.0;  // Total distance traveled in cm

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT);           // Set pin to input
  digitalWrite(2, HIGH);       // Enable pull-up resistor

  // Setting up interrupt
  attachInterrupt(0, ai0, RISING);  // DigitalPin 2 triggers the interrupt on rising edge
}

void loop() {
  // Check if it's time to send data
  if (timeInterval()) {
    sendData();
    resetSampling();
  }
}

void ai0() {
  // Increment the counter for each pulse
  counter++;
}

void sendData() {
  // Calculate RPM
  float rpm = (float(counter) * 60) / pulsesPerRevolution;

  // Print RPM
  Serial.print("RPM = ");
  Serial.print(rpm);

  // Calculate distance traveled in this interval
  float distanceThisInterval = (float(counter) / pulsesPerRevolution) * wheelCircumference;

  // Add interval distance to total distance
  totalDistance += distanceThisInterval;

  Serial.print("\t Distance this interval = ");
  Serial.print(distanceThisInterval);
  Serial.print(" cm");

  // Print total distance traveled
  Serial.print("\t Total distance = ");
  Serial.print(totalDistance);
  Serial.println(" cm");
}

void resetSampling() {
  // Reset counter for the next interval
  counter = 0;
}

boolean timeInterval() {
  unsigned long currentTime = millis();

  // Check if the interval has elapsed
  if (currentTime - lastTime >= interval) {
    lastTime = lastTime + interval;
    return true;
  } else if (currentTime < lastTime) {
    // Handle millis() overflow after ~50 days
    lastTime = 0;
  }
  return false;
}
