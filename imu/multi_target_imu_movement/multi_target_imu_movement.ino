#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define M1_DIR_PIN 22   // Front Left
#define M1_PWM_PIN 4    // Front Left
#define M2_DIR_PIN 24   // Front Right
#define M2_PWM_PIN 5    // Front Right
#define M3_DIR_PIN 26   // Rear Right
#define M3_PWM_PIN 6    // Rear Right
#define M4_DIR_PIN 28   // Rear Left
#define M4_PWM_PIN 7    // Rear Left

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float currentAngle = 0;
float offsetAngle = 0;
bool isFirstReading = true;

const int ROTATION_SPEED = 60;
const float ANGLE_THRESHOLD = 5.0;

// Predefined target angles (Modify these as needed)
float targetQueue[] = {45, 450, -45, -45};  // Set your desired angles
const int totalTargets = sizeof(targetQueue) / sizeof(targetQueue[0]);
int targetIndex = 0;
bool isRotating = false;

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
  Serial.println("System ready. Executing predefined rotations...");
}

void loop() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;

    float rawAngle = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / PI;

    if (rawAngle < 0) {
      rawAngle += 360;
    }

    if (isFirstReading) {
      offsetAngle = rawAngle;
      isFirstReading = false;
      Serial.println("Zero position set!");
    }

    currentAngle = rawAngle - offsetAngle;
    if (currentAngle < 0) currentAngle += 360;
    if (currentAngle >= 360) currentAngle -= 360;

    if (isRotating && targetIndex < totalTargets) {
      float error = targetQueue[targetIndex] - currentAngle;

      if (error > 180) error -= 360;
      if (error < -180) error += 360;

      if (abs(error) < ANGLE_THRESHOLD) {
        stopMotors();
        Serial.print("Target ");
        Serial.print(targetIndex + 1);
        Serial.println(" reached!");

        offsetAngle = rawAngle;  // Reset reference angle
        isRotating = false;

        delay(3000);  // Pause for 3 seconds before moving to the next target

        targetIndex++;  // Move to the next target
      } else {
        if (error < 0) {
          clockwise();
        } else {
          anticlockwise();
        }
      }
    } else if (targetIndex < totalTargets) {
      isRotating = true;  // Start rotation for the next target
    } else {
      Serial.println("All targets completed.");
      while (1); // Stop the loop after finishing all targets
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

void anticlockwise() {
  Serial.println("Moving Counterclockwise");
  digitalWrite(M1_DIR_PIN, LOW);
  analogWrite(M1_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M2_DIR_PIN, HIGH);
  analogWrite(M2_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M3_DIR_PIN, HIGH);
  analogWrite(M3_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M4_DIR_PIN, LOW);
  analogWrite(M4_PWM_PIN, ROTATION_SPEED);
}

void clockwise() {
  Serial.println("Moving Clockwise");
  digitalWrite(M1_DIR_PIN, HIGH);
  analogWrite(M1_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M2_DIR_PIN, LOW);
  analogWrite(M2_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M3_DIR_PIN, LOW);
  analogWrite(M3_PWM_PIN, ROTATION_SPEED);
  digitalWrite(M4_DIR_PIN, HIGH);
  analogWrite(M4_PWM_PIN, ROTATION_SPEED);
}
