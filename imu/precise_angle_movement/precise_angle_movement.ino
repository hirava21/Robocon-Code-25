#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define M1_DIR_PIN 22   // Front Left
#define M1_PWM_PIN 4   // Front Left
#define M2_DIR_PIN 24  // Front Right
#define M2_PWM_PIN 5  // Front Right
#define M3_DIR_PIN 26  // Rear Right
#define M3_PWM_PIN 6  // Rear Right
#define M4_DIR_PIN 28   // Rear Left
#define M4_PWM_PIN 7   // Rear Left
#define true 1
#define false 0

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float currentAngle = 0;
float offsetAngle = 0;
bool isFirstReading = true;
float targetAngle = -135;
const int ROTATION_SPEED = 60; 
const float ANGLE_THRESHOLD = 5.0;  

void setup() {
  Serial.begin(115200);
  

  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT);
  pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M4_PWM_PIN, OUTPUT);
  

  stopMotors();
  

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO085!");
    while (1) delay(10);
  }
  

  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
  
  delay(1000); 
  // Serial.println("System ready. Enter target angle (0-360):");
}

void loop() {
  if (Serial.available() > 0) {
    targetAngle = Serial.parseFloat();
    // Serial.print("New target angle: ");
    // Serial.println(targetAngle);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {


    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;
    

    float rawAngle = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / PI;
    
// Serial.println(rawAngle, 6);
    if (rawAngle < 0) {
      rawAngle += 360;
    }
    if (isFirstReading) {
      offsetAngle = rawAngle;
      isFirstReading = false;
      // Serial.println("Zero position set!");
    }
    currentAngle = rawAngle - offsetAngle;
    if (currentAngle < 0) currentAngle += 360;
    if (currentAngle >= 360) currentAngle -= 360;
    float error = targetAngle - currentAngle;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    if (abs(error) < ANGLE_THRESHOLD) {
      stopMotors();
      // Serial.println("Target reached!");
    }
    else {
      if (targetAngle <= 180) {
        anticlockwise();
      } else {
      clockwise();
      }
    }
  }  
  delay(10); 
}

void stopMotors() {
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(M3_PWM_PIN, 0);
  analogWrite(M4_PWM_PIN, 0);
}
void anticlockwise(){
Serial.println("Moving Counterclockwise");
digitalWrite(M1_DIR_PIN, LOW);   // Front Left
analogWrite(M1_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M2_DIR_PIN, HIGH);   // Front Right
analogWrite(M2_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M3_DIR_PIN, HIGH);   // Rear Right
analogWrite(M3_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M4_DIR_PIN, LOW);   // Rear Left
analogWrite(M4_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
}
void clockwise(){
Serial.println("Moving Clockwise");
digitalWrite(M1_DIR_PIN, HIGH);   // Front Left
analogWrite(M1_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M2_DIR_PIN, LOW);   // Front Right
analogWrite(M2_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M3_DIR_PIN, LOW);   // Rear Right
analogWrite(M3_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
digitalWrite(M4_DIR_PIN, HIGH);   // Rear Left
analogWrite(M4_PWM_PIN, ROTATION_SPEED); // ✅ Fixed
}