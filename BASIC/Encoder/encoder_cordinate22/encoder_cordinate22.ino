// ---- Motor Pins ----
#define FL_pwm 14
#define FR_pwm 15
#define RL_pwm 19
#define RR_pwm 18
#define FL_dir 30
#define FR_dir 31
#define RL_dir 33
#define RR_dir 32

// ---- Encoder Pins ----
#define ENCODER_FL_A 0
#define ENCODER_FL_B 1
#define ENCODER_FR_A 2
#define ENCODER_FR_B 3
#define ENCODER_RL_A 6
#define ENCODER_RL_B 7
#define ENCODER_RR_A 4
#define ENCODER_RR_B 5

int motorSpeed = 100;

// ---- Encoder Values ----
volatile long encoderCount_FL = 0;
volatile long encoderCount_FR = 0;
volatile long encoderCount_RL = 0;
volatile long encoderCount_RR = 0;

const float wheelDiameter = 10.0;  // cm
const float wheelCircumference = 3.1416 * wheelDiameter;
const int encoderPPR = 7;
const float gearRatio = 19.2;
const float pulsesPerRotation = encoderPPR * gearRatio;
const float distancePerPulse = wheelCircumference / pulsesPerRotation;
const float correctionFactorY = 0.707317;  // for forward/reverse
const float correctionFactorX = 0.80645;  // for right/left


void encoderFL_ISR() { encoderCount_FL++; }
void encoderFR_ISR() { encoderCount_FR++; }
void encoderRL_ISR() { encoderCount_RL++; }
void encoderRR_ISR() { encoderCount_RR++; }

void setup() {
  Serial.begin(115200);
  pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);

  pinMode(ENCODER_FL_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), encoderFL_ISR, RISING);
  pinMode(ENCODER_FR_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), encoderFR_ISR, RISING);
  pinMode(ENCODER_RL_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_RL_A), encoderRL_ISR, RISING);
  pinMode(ENCODER_RR_A, INPUT); attachInterrupt(digitalPinToInterrupt(ENCODER_RR_A), encoderRR_ISR, RISING);

  Serial.println("Setup Complete");
}

void loop() {
  moveToCoordinate(290, 250);   // forward 290 cm, then right 250 cm
  delay(5000);
  // moveToCoordinate(-100, -50); // reverse 100 cm, then left 50 cm
  // delay(10000);
}

void moveToCoordinate(float yDistance, float xDistance) {
  if (yDistance != 0) {
    resetEncoders();
    if (yDistance > 0) forward();
    else reverse();
    while (getAverageDistance() < abs(yDistance) * correctionFactorY) delay(10);
    stop();
    delay(300);
  }

  if (xDistance != 0) {
    resetEncoders();
    if (xDistance > 0) right();
    else left();
    while (getAverageDistance() < abs(xDistance) * correctionFactorX) delay(10);
    stop();
    delay(300);
  }
}


void resetEncoders() {
  encoderCount_FL = 0;
  encoderCount_FR = 0;
  encoderCount_RL = 0;
  encoderCount_RR = 0;
}

float getAverageDistance() {
  float avgCount = (encoderCount_FL + encoderCount_FR + encoderCount_RL + encoderCount_RR) / 4.0;
  return avgCount * distancePerPulse;
}

// ---- Movement Functions ----
void forward() {
  digitalWrite(FL_dir, HIGH); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, HIGH); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, HIGH); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, HIGH); analogWrite(RL_pwm, motorSpeed);
}

void reverse() {
  digitalWrite(FL_dir, LOW); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, LOW); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, LOW); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, LOW); analogWrite(RL_pwm, motorSpeed);
}

void right() {
  digitalWrite(FL_dir, HIGH); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, LOW);  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, HIGH); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, LOW);  analogWrite(RL_pwm, motorSpeed);
}

void left() {
  digitalWrite(FL_dir, LOW);  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, HIGH); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, LOW);  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, HIGH); analogWrite(RL_pwm, motorSpeed);
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
}
