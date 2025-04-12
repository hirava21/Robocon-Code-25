#include "BLDC.h"

const int escPin = 28;
BLDC bldc(escPin);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Example: gradually increase speed
  for (int speed = 1000; speed <= 1500; speed += 10) {
    bldc.setSpeed(2000);
    delay(20);
  }

  delay(1000);

  // Gradually decrease speed
  for (int speed = 1500; speed >= 1000; speed -= 10) {
    bldc.setSpeed(speed);
    delay(20);
  }

  delay(1000);
}
