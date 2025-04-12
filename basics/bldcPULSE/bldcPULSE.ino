const int escPin = 28; // Use any PWM-capable pin

void setup() {
  pinMode(escPin, OUTPUT);
  Serial.begin(9600);

  // Initial ESC arm sequence
  Serial.println("Arming ESC...");
  for (int i = 0; i < 50; i++) {
    writePulse(escPin, 1000); // Send minimum signal
    delay(20);
  }
  Serial.println("ESC Armed!");
}

void loop() {
  // Example: ramp up the speed gradually
  for (int speed = 1000; speed <= 2000; speed += 10) {
    writePulse(escPin, speed);
    delay(20);
  }

  delay(1000);

  // Ramp down
  for (int speed = 2000; speed >= 1000; speed -= 10) {
    writePulse(escPin, speed);
    delay(20);
  }

  delay(1000);
}

// Sends a PWM pulse of given width (in microseconds)
void writePulse(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delay(20 - (pulseWidth / 1000)); // maintain ~50Hz (20ms period)
}
 