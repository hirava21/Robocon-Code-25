#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// === IMU Setup ===
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

const float rotationThreshold = 0.05;
float angle = 0.0;
float initialHeading = 0.0;  // NEW: store angle after forward
unsigned long previousTime = 0;

// === Basket Position ===
const float x_basket = 380.0; // cm
const float y_basket = 720.0; // cm

// === Encoder Pins & Counters ===
volatile long countFL = 0, countFR = 0, countRL = 0, countRR = 0;

// Wheel specs
const int ppr = 7;
const float gearRatio = 19.2;
const float pulsesPerRotation = ppr * gearRatio;

const float wheelDiameter = 10.0; // cm
const float wheelCircumference = PI * wheelDiameter;
const float distancePerPulse = wheelCircumference / pulsesPerRotation;

float x_bot = 0;
float y_bot = 0;

bool movedForward = false;
bool rotatedToBasket = false;

// === Movement Control ===
int motorSpeed = 100;

// === Pin Definitions ===
#define FL_pwm 14
#define FR_pwm 15
#define RL_pwm 19
#define RR_pwm 18

#define FL_dir 30
#define FR_dir 31
#define RL_dir 33
#define RR_dir 32

void initializeMotors() {
  pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);
}

void forward() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
}

void clockwise() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
}

void anticlockwise() {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, motorSpeed);
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
}

// === IMU Setup ===
void initializeSensor() {
  Wire1.begin();
  if (!bno.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  Serial.println("BNO08x Found!");
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED);
}

void processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) return;

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    if (abs(zRotation) > rotationThreshold) {
      angle += zRotation * deltaTime;
    }
  }
}

// === Encoder ISRs ===
void ISR_FL() { digitalRead(0) == HIGH ? countFL++ : countFL--; }
void ISR_FR() { digitalRead(3) == HIGH ? countFR++ : countFR--; }
void ISR_RL() { digitalRead(7) == HIGH ? countRL++ : countRL--; }
void ISR_RR() { digitalRead(5) == HIGH ? countRR++ : countRR--; }

void initializeEncoders() {
  pinMode(1, INPUT_PULLUP); pinMode(0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP); pinMode(7, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP); pinMode(5, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(1), ISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(2), ISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(6), ISR_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(4), ISR_RR, RISING);
}

void updateBotPosition() {
  noInterrupts();
  long fl = countFL, fr = countFR, rl = countRL, rr = countRR;
  interrupts();

  float distY = -((fl + rr) / 2.0) * distancePerPulse;
  float distX = -((fr + rl) / 2.0) * distancePerPulse;

  x_bot = distX;
  y_bot = distY;

  Serial.print("X: "); Serial.print(x_bot);
  Serial.print("\tY: "); Serial.println(y_bot);
}

void rotateToFaceBasket() {
  float dx = x_basket - x_bot;
  float dy = y_basket - y_bot;

  float desiredAngle = atan2(dy, dx); // ✅ correct order
  float currentAngle = angle - initialHeading;

  desiredAngle = normalizeAngle(desiredAngle);
  currentAngle = normalizeAngle(currentAngle);

  float error = normalizeAngle(desiredAngle - currentAngle);
  float errorDeg = error * 180.0 / PI;

  Serial.print("desiredAngle: "); Serial.println(desiredAngle * 180 / PI);
  Serial.print("currentAngle: "); Serial.println(currentAngle * 180 / PI);
  Serial.print("error: "); Serial.println(errorDeg);

  if (abs(errorDeg) < 3) {
    stop();
    rotatedToBasket = true;
    Serial.println("Bot facing basket!");
    return;
  }

  if (errorDeg > 0) clockwise();
  else anticlockwise();
}
float normalizeAngle(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

// === Setup & Loop ===
void setup() {
  Serial.begin(115200);
  initializeSensor();
  initializeMotors();
  initializeEncoders();
  previousTime = millis();
}

void loop() {
  processGyroscopeData();

  if (!movedForward) {
    forward();
    updateBotPosition();

    if (y_bot >= 100.0) {
      stop();
      countFL = countFR = countRL = countRR = 0;
      movedForward = true;
      initialHeading = angle;  // ✅ Save actual heading after moving forward
      delay(500);
    }
  }

  if (movedForward && !rotatedToBasket) {
    rotateToFaceBasket();
  }

  delay(50);
}
