#include <Wire.h>

int FL_pwm = 2, FR_pwm = 4, RL_pwm = 8, RR_pwm = 6; // Motor PWM pins
int FL_dir = 3, FR_dir = 5, RL_dir = 9, RR_dir = 7; // Motor direction pins
int motorSpeed = 255; // Maximum motor speed (0-255)

void setup() {
  pinMode(FL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT);

  Wire.begin(8); // Initialize as I2C slave with address 8
  Wire.onReceive(receiveEvent); // Set callback for receiving data
}

void loop() {
  // Nothing to do here; everything is handled in receiveEvent
}

void receiveEvent(int bytes) {
  String command = "";
  while (Wire.available()) {
    char c = Wire.read(); // Read each byte
    command += c;
  }
  command.trim();

  if (command == "FORWARD") {
    forward();
  } else if (command == "REVERSE") {
    reverse();
  } else if (command == "LEFT") {
    left();
  } else if (command == "RIGHT") {
    right();
  } else if (command == "CLOCKWISE") {
    clockwise();
  } else if (command == "ANTICLOCKWISE") {
    anticlockwise();
  } else if (command == "STOP") {
    stop();
  }
}

void forward() {
  setMotors(motorSpeed, HIGH, motorSpeed, HIGH, motorSpeed, HIGH, motorSpeed, HIGH);
}

void reverse() {
  setMotors(motorSpeed, LOW, motorSpeed, LOW, motorSpeed, LOW, motorSpeed, LOW);
}

void right() {
  setMotors(motorSpeed, HIGH, motorSpeed, LOW, motorSpeed, LOW, motorSpeed, HIGH);
}

void left() {
  setMotors(motorSpeed, LOW, motorSpeed, HIGH, motorSpeed, HIGH, motorSpeed, LOW);
}

void stop() {
  setMotors(0, LOW, 0, LOW, 0, LOW, 0, LOW);
}

void clockwise() {
  setMotors(motorSpeed, HIGH, motorSpeed, LOW, motorSpeed, HIGH, motorSpeed, LOW);
}

void anticlockwise() {
  setMotors(motorSpeed, LOW, motorSpeed, HIGH, motorSpeed, LOW, motorSpeed, HIGH);
}

void setMotors(int fl_speed, int fl_dir, int fr_speed, int fr_dir, int rr_speed, int rr_dir, int rl_speed, int rl_dir) {
  digitalWrite(FL_dir, fl_dir);
  analogWrite(FL_pwm, fl_speed);

  digitalWrite(FR_dir, fr_dir);
  analogWrite(FR_pwm, fr_speed);

  digitalWrite(RR_dir, rr_dir);
  analogWrite(RR_pwm, rr_speed);

  digitalWrite(RL_dir, rl_dir);
  analogWrite(RL_pwm, rl_speed);
}
