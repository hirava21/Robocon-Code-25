#define X_ENCODER_A 3  
#define X_ENCODER_B 2
#define Y_ENCODER_A 19
#define Y_ENCODER_B 18
int FL_pwm = 8; 
int FR_pwm = 10; 
int RL_pwm = 6; 
int RR_pwm = 12;

int FL_dir = 9; 
int FR_dir = 11; 
int RL_dir = 7; 
int RR_dir = 13; 

int motorSpeed = 25;
const float PULSES_PER_REVOLUTION = 600.0;
const float WHEEL_DIAMETER = 10;         // cm
const float CM_PER_PULSE = (PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;

volatile long x_encoder_count = 0;
volatile long y_encoder_count = 0;
float x_position = 0.0;
float y_position = 0.0;

float x_calibration = 1.0;
float y_calibration = 1.0;

bool forwardCompleted = false;  // Flag to check forward motion completion
bool rightCompleted = false;    // Flag to check right motion completion

void setup() {
  Serial.begin(9600);
  
  pinMode(X_ENCODER_A, INPUT_PULLUP);
  pinMode(X_ENCODER_B, INPUT_PULLUP);
  pinMode(Y_ENCODER_A, INPUT_PULLUP);
  pinMode(Y_ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(X_ENCODER_A), x_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_ENCODER_A), y_encoder_isr, CHANGE);
  
  Serial.println("Holonomic Drive Position Tracking Started");
  Serial.println("Position Format: (X cm, Y cm)");
    pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
    pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
    pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);
    pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);
}

void loop() {
  // Update position values
  x_position = x_encoder_count * CM_PER_PULSE * x_calibration;
  y_position = y_encoder_count * CM_PER_PULSE * y_calibration;
  
  Serial.print("( X:- ");
  Serial.print(x_position, 2);
  Serial.print(", Y:- ");
  Serial.print(y_position, 2);
  Serial.println(")");

  // Move forward until x_position reaches 30 cm
  if (!forwardCompleted) {
    if (x_position < 30) {
      forward();
    } else {
      stop();
      forwardCompleted = true;  // Mark forward movement as complete
      delay(1000); // Short pause before moving right
    }
  }

  // Move right until y_position reaches 30 cm
  if (forwardCompleted && !rightCompleted) {
    if (y_position < 30) {
      right();
    } else {
      stop();
      rightCompleted = true; // Mark right movement as complete
      Serial.println("Movement completed");
    }
  }

  delay(100);
}

void x_encoder_isr() {
  int a = digitalRead(X_ENCODER_A);
  int b = digitalRead(X_ENCODER_B);
  
  if (a == b) {
    x_encoder_count++;
  } else {
    x_encoder_count--;
  }
}

void y_encoder_isr() {
  int a = digitalRead(Y_ENCODER_A);
  int b = digitalRead(Y_ENCODER_B);
  
  if (a == b) {
    y_encoder_count++;
  } else {
    y_encoder_count--;
  }
}

void forward() { 
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  // Serial.println("Moving forward");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  // Serial.println("Stopped");
}

void right() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  // Serial.println("Moving right");
}
