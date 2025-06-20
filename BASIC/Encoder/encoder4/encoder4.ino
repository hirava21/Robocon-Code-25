#include <Arduino.h>

// ---- Encoder Pin Definitions ----
const int encoderFL_A = 1; // Y-axis
const int encoderFL_B = 0;

const int encoderFR_A = 2; // X-axis
const int encoderFR_B = 3;

const int encoderRL_A = 6; // X-axis
const int encoderRL_B = 7;

const int encoderRR_A = 4; // Y-axis
const int encoderRR_B = 5;

// ---- Encoder Pulse Counters ----
volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

// ---- Wheel & Encoder Specs ----
const int ppr = 7;
const float gearRatio = 19.2;
const float pulsesPerRotation = ppr * gearRatio;

const float wheelDiameter = 10.0; // cm
const float wheelCircumference = PI * wheelDiameter;
const float distancePerPulse = wheelCircumference / pulsesPerRotation;

// ---- ISRs ----
void ISR_FL() {
  digitalRead(encoderFL_B) == HIGH ? countFL++ : countFL--;
}
void ISR_FR() {
  digitalRead(encoderFR_B) == HIGH ? countFR++ : countFR--;
}
void ISR_RL() {
  digitalRead(encoderRL_B) == HIGH ? countRL++ : countRL--;
}
void ISR_RR() {
  digitalRead(encoderRR_B) == HIGH ? countRR++ : countRR--;
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderFL_A, INPUT_PULLUP);
  pinMode(encoderFL_B, INPUT_PULLUP);
  pinMode(encoderFR_A, INPUT_PULLUP);
  pinMode(encoderFR_B, INPUT_PULLUP);
  pinMode(encoderRL_A, INPUT_PULLUP);
  pinMode(encoderRL_B, INPUT_PULLUP);
  pinMode(encoderRR_A, INPUT_PULLUP);
  pinMode(encoderRR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderFL_A), ISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderFR_A), ISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRL_A), ISR_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRR_A), ISR_RR, RISING);
}

void loop() {
  long fl, fr, rl, rr;

  noInterrupts(); // Snapshot encoder values safely
  fl = countFL;
  fr = countFR;
  rl = countRL;
  rr = countRR;
  interrupts();

  float distY = ((fl + rr) / 2.0) * distancePerPulse;
  float distX = ((fr + rl) / 2.0) * distancePerPulse;

  Serial.print("Distance X (cm): ");
  Serial.print(distX);
  Serial.print("\tDistance Y (cm): ");
  Serial.println(distY);

  delay(200);
}
