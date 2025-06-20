#include <Arduino.h>

const int encoderXPinA = 6;
const int encoderXPinB = 7;
const int encoderYPinA = 1;
const int encoderYPinB = 0;

volatile long pulseCountX = 0;
volatile long pulseCountY = 0;
const int ppr = 7;

const float wheelDiameter = 10.0; // cm
const float wheelCircumference = PI * wheelDiameter; // ~31.4159 cm
const float distancePerPulse = wheelCircumference / ppr;

void countPulseX();
void countPulseY();

void setup() {
  Serial.begin(9600);

  // Encoder pins
  pinMode(encoderXPinA, INPUT_PULLUP);
  pinMode(encoderXPinB, INPUT_PULLUP);
  pinMode(encoderYPinA, INPUT_PULLUP);
  pinMode(encoderYPinB, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderXPinA), countPulseX, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderYPinA), countPulseY, RISING);
}

void loop() {
  float distanceX, distanceY;

  noInterrupts();
  long pulsesX = pulseCountX;
  long pulsesY = pulseCountY;
  interrupts();

  distanceX = pulsesX * distancePerPulse;
  distanceY = pulsesY * distancePerPulse;
  Serial.print("Distance X: ");
  Serial.print(distanceX / 19.2);
  Serial.print(" cm\t");

  Serial.print("Distance Y: ");
  Serial.print(distanceY / 19.2);
  Serial.println(" cm");

  delay(200);
}

void countPulseX() {
  if (digitalRead(encoderXPinB) == HIGH) {
    pulseCountX++;
  } else {
    pulseCountX--;
  }
}
\
void countPulseY() {
  if (digitalRead(encoderYPinB) == HIGH) {
    pulseCountY++;
  } else {
    pulseCountY--;
  }
}
