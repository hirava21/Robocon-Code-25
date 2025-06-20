// ---- Pin Definitions ----
#define FL_dir 30
#define FL_pwm 14
#define FR_dir 31
#define FR_pwm 15
#define RR_dir 32
#define RR_pwm 18
#define RL_dir 33
#define RL_pwm 19

// ---- Encoder Pins ----
#define ENCODER_FL_A 0
#define ENCODER_FL_B 1
#define ENCODER_FR_A 2
#define ENCODER_FR_B 3
#define ENCODER_RL_A 6
#define ENCODER_RL_B 7
#define ENCODER_RR_A 4
#define ENCODER_RR_B 5

#include <Wire.h>

// ---- Bot Geometry ----
const float botWidth = 48.5;  // cm
const float botLength = 48.5; // cm

// ---- Motor & Movement Parameters ----
int motorSpeed = 100;
const float wheelDiameter = 10.0;  // cm
const float wheelCircumference = 3.1416 * wheelDiameter;
const int encoderPPR = 7;
const float gearRatio = 19.2;
const float pulsesPerRotation = encoderPPR * gearRatio;
const float distancePerPulse = wheelCircumference / pulsesPerRotation;

// ---- Distance Configuration ----
const float forwardDistance = 100.0;  // in cm
const float leftDistance = 50.0;      // in cm

// ---- Encoder Counts ----
volatile long encoderCount_FL = 0;
volatile long encoderCount_FR = 0;
volatile long encoderCount_RL = 0;
volatile long encoderCount_RR = 0;

// ---- Movement State Machine ----
bool step1_done = false;
bool step2_done = false;
unsigned long waitStartTime = 0;
bool isMovingLeft = false;

// ---- Encoder ISRs ----
void encoderFL_A_ISR() { encoderCount_FL++; }
void encoderFR_A_ISR() { encoderCount_FR++; }
void encoderRL_A_ISR() { encoderCount_RL++; }
void encoderRR_A_ISR() { encoderCount_RR++; }

void setup() {
  Serial.begin(115200);

  pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);

  pinMode(ENCODER_FL_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), encoderFL_A_ISR, RISING);
  pinMode(ENCODER_FR_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), encoderFR_A_ISR, RISING);
  pinMode(ENCODER_RL_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_RL_A), encoderRL_A_ISR, RISING);
  pinMode(ENCODER_RR_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_RR_A), encoderRR_A_ISR, RISING);

  Serial.println("Setup Complete");
}

void loop() {
  if (!step1_done) {
    resetEncoders();
    isMovingLeft = false;
    forward();
    while (getCenterDistance() < forwardDistance) {
      delay(10);
    }
    stop();
    waitStartTime = millis();
    step1_done = true;
  }
  else if (step1_done && !step2_done && millis() - waitStartTime > 1000) {
    resetEncoders();
    isMovingLeft = true;
    left();
    while (getCenterDistance() < leftDistance) {
      delay(10);
    }
    stop();
    step2_done = true;
    Serial.println("Completed All Steps");
  }
}

float getCenterDistance() {
  float avgCount = (encoderCount_FL + encoderCount_FR + encoderCount_RL + encoderCount_RR) / 4.0;
  float baseDistance = avgCount * distancePerPulse;
  if (isMovingLeft) {
    return baseDistance * 0.7071;  // scaled for X-drive lateral movement (1/sqrt(2))
  } else {
    return baseDistance;
  }
}

void resetEncoders() {
  encoderCount_FL = 0;
  encoderCount_FR = 0;
  encoderCount_RL = 0;
  encoderCount_RR = 0;
}

void forward() {
  digitalWrite(FL_dir, HIGH); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, HIGH); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, HIGH); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, HIGH); analogWrite(RL_pwm, motorSpeed);
  Serial.println("forward");
}

void reverse() {
  digitalWrite(FL_dir, LOW); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, LOW); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, LOW); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, LOW); analogWrite(RL_pwm, motorSpeed);
  Serial.println("reverse");
}

void right() {
  digitalWrite(FL_dir, HIGH); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, LOW); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, HIGH); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, LOW); analogWrite(RL_pwm, motorSpeed);
  Serial.println("right");
}

void left() {
  digitalWrite(FL_dir, LOW); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, HIGH); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, LOW); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, HIGH); analogWrite(RL_pwm, motorSpeed);
  Serial.println("left");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("stop");
}
