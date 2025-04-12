#include "BLDC.h"

BLDC::BLDC(int pin) {
  escPin = pin;
  pinMode(escPin, OUTPUT);
}

void BLDC::writePulse(int pulseWidth) {
  digitalWrite(escPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(escPin, LOW);
  delay(20 - (pulseWidth / 1000)); // maintain ~50Hz
}

void BLDC::setSpeed(int pulseWidth) {
  writePulse(pulseWidth);
}
void BLDC::up() {
  for (int speed = 1000; speed <= 2000; speed += 10) {
    writePulse(speed);
    delay(20);
  }
}

void BLDC::down() {
  for (int speed = 2000; speed >= 1000; speed -= 10) {
    writePulse(speed);
    delay(20);
  }
}