#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

// PID Constants
float Kp = 1.8;    // Proportional gain
float Ki = 0.1;    // Integral gain
float Kd = 1.0;    // Derivative gain

const float rotationThreshold = 0.05;  
const float angleTolerance = 0.05;     

float angle = 0.0;  
unsigned long previousTime = 0;

float prevError = 0;
float integral = 0;
float initialYaw = 0;
float deg = 0.0;

int BS = 50;              // Base speed
int correctionFactor = 10; // Minimum correction speed
int maxSpeed = 150;       // Maximum speed after ramping

int FL_pwm = 4, FR_pwm = 5, RL_pwm = 7, RR_pwm = 6;
int FL_dir = 22, FR_dir = 24, RL_dir = 28, RR_dir = 26;

// Deadzone & Ramping Variables
float deadzone = 5;       // Ignore minor errors within ±3°
int rampTime = 500;      // Time to reach base speed (in ms)
unsigned long startTime;  // Start time for ramping

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
  previousTime = millis();  

  pinMode(FR_pwm, OUTPUT); pinMode(FR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT); pinMode(RR_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT); pinMode(RL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT); pinMode(FL_dir, OUTPUT);

  startTime = millis(); // Start ramping time
}

void loop() {
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
    float deg = angle * (180 / PI);

    Serial.print("Angle displaced: ");
    Serial.print(deg);
    Serial.println(" DEG");

    // Normalize angle within [-180, 180]
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;

    // PID Calculations
    float error = deg;
    integral = (abs(error) > deadzone) ? integral + error : 0;  // Reset integral in deadzone
    float derivative = error - prevError;
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
    prevError = error;

    // Convert PID output to orientation correction
    float orientationCorrection = constrain(pidOutput, -150, 150);

    // Speed ramping logic
    unsigned long elapsedTime = millis() - startTime;
    int currentSpeed = map(min(elapsedTime, rampTime), 0, rampTime, BS, maxSpeed);

    float desired_vy = currentSpeed;  
    float correction_vx = 0;          

    float wheel1 = desired_vy + correction_vx + orientationCorrection;
    float wheel2 = desired_vy - correction_vx - orientationCorrection;
    float wheel3 = desired_vy + correction_vx + orientationCorrection;
    float wheel4 = desired_vy - correction_vx - orientationCorrection;

    // Constrain wheel speeds to motor limits
    wheel1 = constrain(wheel1, -150, 150);
    wheel2 = constrain(wheel2, -150, 150);
    wheel3 = constrain(wheel3, -150, 150);
    wheel4 = constrain(wheel4, -150, 150);

    // Apply wheel speeds
    drive(wheel1, wheel2, wheel3, wheel4);
  }
}
void drive(int w1, int w2, int w3, int w4) {
  analogWrite(FL_pwm, abs(w1));
  digitalWrite(FL_dir, w1 >= 0);
  
  analogWrite(FR_pwm, abs(w2));
  digitalWrite(FR_dir, w2 >= 0);

  analogWrite(RL_pwm, abs(w3));
  digitalWrite(RL_dir, w3 >= 0);

  analogWrite(RR_pwm, abs(w4));
  digitalWrite(RR_dir, w4 >= 0);
}
