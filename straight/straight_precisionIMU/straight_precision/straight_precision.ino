#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

const float rotationThreshold = 0.02;  
const float angleTolerance = 0.03;       
int BS = 150, correctionFactor = 10;  
int FL_pwm = 4, FR_pwm = 5, RL_pwm = 7, RR_pwm = 6;
int FL_dir = 22, FR_dir = 24, RL_dir = 28, RR_dir = 26;  

float angle = 0.0;  
unsigned long previousTime = 0;
unsigned long startTime = 0;  
const unsigned long runDuration = 5000; // Run for 3 seconds

// Define missing variables
int currentLS = 0, currentRS = 0;  // Initialize current speed to zero
const int stepSize = 5;  // Speed increment step size
const int delayTime = 50;  // Delay time for gradual speed adjustment

void initializeSensor() {
  Wire.begin();
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);  
  }
  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);  
  }
}

void setup() {
  Serial.begin(115200);
  initializeSensor();
  previousTime = micros();
  startTime = millis();  // Store the start time

  pinMode(FR_pwm, OUTPUT); pinMode(FR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT); pinMode(RR_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT); pinMode(RL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT); pinMode(FL_dir, OUTPUT);
}

void loop() {
  // if (millis() - startTime < runDuration) {
    processGyroscopeData();
    delay(100);
}
//   else {
//     stop();  // Stop the bot after 5 seconds
//     while (1); // Halt execution
//   }
// }

void processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();

    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    if (abs(zRotation) > rotationThreshold) {
      angle += zRotation * deltaTime;
    }

    Serial.print("Angle displaced: ");
    Serial.print(angle);
    Serial.print(" rad, ");
    
    Serial.print("Angle displaced: ");
    Serial.print(angle * 180.0 / PI);  // Convert to degrees
    Serial.println(" degrees, ");


    if (abs(angle) > angleTolerance) {
      if (angle > 0) {
        Serial.print("Correcting Right (Slow down Right)");
        Serial.println("Bot should go Clockwise to correct angle");
        forward(BS, BS - correctionFactor);
      } else {
        Serial.print("Bot should go Anticlockwise to correct angle");
        Serial.println("Correcting Left (Slow down Left)");
        forward(BS - correctionFactor, BS);
      }
    } else {
      Serial.println("Angle is near zero, no correction needed.");
      forward(BS, BS);
    }
  }
}

void forward(int targetLS, int targetRS) {
  while (currentLS != targetLS || currentRS != targetRS) {
    // Adjust Left Speed
    if (currentLS < targetLS) {
      currentLS = min(currentLS + stepSize, targetLS);
    } else if (currentLS > targetLS) {
      currentLS = max(currentLS - stepSize, targetLS);
    }

    // Adjust Right Speed
    if (currentRS < targetRS) {
      currentRS = min(currentRS + stepSize, targetRS);
    } else if (currentRS > targetRS) {
      currentRS = max(currentRS - stepSize, targetRS);
    }

    // Apply PWM
    digitalWrite(FL_dir, 1);
    analogWrite(FL_pwm, currentLS);
    digitalWrite(FR_dir, 1);
    analogWrite(FR_pwm, currentRS);
    digitalWrite(RR_dir, 1);
    analogWrite(RR_pwm, currentRS);
    digitalWrite(RL_dir, 1);
    analogWrite(RL_pwm, currentLS);

    delay(delayTime);  // Wait before next adjustment
  }
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stop");
}
