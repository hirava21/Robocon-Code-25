#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Motor Pins
int PWM1 = 8; int DIR1 = 9;
int PWM2 = 10; int DIR2 = 11;
int PWM3 = 12; int DIR3 = 13;
int PWM4 = 6; int DIR4 = 7;

int motorSpeed = 50;
BNO080 myIMU;

float initialYaw = 0;
float adjustedYaw = 0;
float throttleTarget = 0;
float mappingTargetAngle = 0;
bool throttleHandled = false;

// Prototypes
void jsm(int axisRX, int axisY, int throttle, int brake);
void bnoval();
void forward();
void reverse();
void left();
void right();
void clockwise();
void anticlockwise();
void stop();
void handleThrottle();
void mapping();

void setup() {
  Serial.begin(9600); // Monitor
  Serial1.begin(9600); // UART communication with ESP32

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    adjustedYaw = yaw - initialYaw;
  }

  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n'); // Read joystick, D-pad, throttle, and brake data
    int delimiter1 = data.indexOf(',');
    int delimiter2 = data.indexOf(',', delimiter1 + 1);
    int delimiter3 = data.indexOf(',', delimiter2 + 1);
    int delimiter4 = data.indexOf(',', delimiter3 + 1);
    if (delimiter1 != -1 && delimiter4 != -1) {
      int axisY = data.substring(0, delimiter1).toInt();
      int axisRX = data.substring(delimiter1 + 1, delimiter2).toInt();
      int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
      int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
      int brake = data.substring(delimiter4 + 1).toInt();

      jsm(axisRX, axisY, throttle, brake);
      bnoval();
    }
  }
}

void bnoval() {
  if (adjustedYaw > 180) {
    adjustedYaw -= 360;
  } else if (adjustedYaw < -180) {
    adjustedYaw += 360;
  }

  Serial.print("Adjusted Yaw: ");
  Serial.println(adjustedYaw, 2);
}

void jsm(int axisRX, int axisY, int throttle, int brake) {
  if (axisY <= -25) {
    motorSpeed = map(axisY, -25, -512, 0, 50);
    forward();
  } else if (axisY >= 25) {
    motorSpeed = map(axisY, 25, 512, 0, 50);
    reverse();
  }
  if (axisRX <= -25) {
    motorSpeed = map(axisRX, -25, -512, 0, 50);
    left();
  } else if (axisRX >= 25) {
    motorSpeed = map(axisRX, 25, 512, 0, 50);
    right();
  }
  if (throttle > 0) {
    int val = map(throttle, 0, 1020, 0, 1);
    if (throttle == 1 && !throttleHandled) {
      handleThrottle();
    } else {
      mapping();
    }
  }
  if (brake > 0) {
    motorSpeed = map(brake, 0, 1024, 0, 50);
    anticlockwise();
  }
  if (axisY > -25 && axisY < 25 && axisRX > -25 && axisRX < 25 && throttle == 0 && brake == 0) {
    stop();
  }
}
void forward(){
digitalWrite(DIR1, HIGH);
analogWrite(PWM1, motorSpeed);
digitalWrite(DIR2, HIGH);
analogWrite(PWM2, motorSpeed);
digitalWrite(DIR3, HIGH);
analogWrite(PWM3, motorSpeed);
digitalWrite(DIR4, HIGH);
analogWrite(PWM4, motorSpeed);
Serial.println("Forward");
}
void reverse(){
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM4, motorSpeed);
   Serial.println("Reverse");
}
void left(){
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM4, motorSpeed);
  Serial.println("Left");
}
void right(){
digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
Serial.println("Right");
}
void clockwise(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
  Serial.println("Clockwise");
}
void anticlockwise(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
  Serial.println("Anticlockwise"); 
}
void stop(){
  analogWrite(PWM1, 0); analogWrite(PWM2, 0);
  analogWrite(PWM3, 0); analogWrite(PWM4, 0);
  Serial.println("STOP");
}
void handleThrottle(){
  if (adjustedYaw < (throttleTarget -5)){
    clockwise();
    Serial.print("clck 45 wala");
  }
else if (adjustedYaw > (throttleTarget +5)){
  anticlockwise();
  Serial.print("anticlk 45 wala");
} else {
  mappingTargetAngle = adjustedYaw;
  throttleHandled = true;
  int throttle =0;
}
  }
void mapping(){
  if (adjustedYaw > (mappingTargetAngle - 5) && adjustedYaw < (mappingTargetAngle + 5)) {
    Serial.println("Moving straight");
    forward();
  } else if (adjustedYaw >= (mappingTargetAngle + 10)) {
    Serial.println("Turning right");
    clockwise();
  } else if (adjustedYaw <= (mappingTargetAngle - 10)) {
    Serial.println("Turning left");
    anticlockwise();
  } else {
    Serial.println("Stopping");
    stop();
  }
}
