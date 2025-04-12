#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

float Kp = 3;
float Ki = 0;
float Kd = 0;



const float rotationThreshold = 0.05;
const float angleTolerance = 0.05;

float angle = 0.0;
unsigned long previousTime = 0;

float prevError = 0;
float integral = 0;
float maxIntegral = 50;

float deg = 0.0;

int BS = 50;
int maxSpeed = 200;
int correctionFactor = 10;

int FL_pwm = 4, FR_pwm = 5, RL_pwm = 7, RR_pwm = 6;
int FL_dir = 22, FR_dir = 24, RL_dir = 28, RR_dir = 26;

float deadzone = 5;
int rampTime = 2000;
int rampDownDuration = 2000;
unsigned long startTime;

unsigned long moveDuration = 5500; // Total movement duration (ms)

bool isMoving = true;
unsigned long moveStartTime = 0;

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

  startTime = millis();
  angle = 0.0;
}

void loop() {
  if (isMoving && moveStartTime == 0) {
    moveStartTime = millis();
  }

  unsigned long currentMillis = millis();
  unsigned long elapsedTotal = currentMillis - moveStartTime;

  if (isMoving && elapsedTotal >= moveDuration) {
    stop();
    isMoving = false;
    return;
  }

  if (!isMoving) return;

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
    if (deg > 180) deg -= 360;
    if (deg < -180) deg += 360;

    float error = deg;

    if (abs(error) > deadzone) {
      integral += error * deltaTime;
      integral = constrain(integral, -maxIntegral, maxIntegral);
    } else {
      integral = 0;
    }

    float derivative = (error - prevError) / deltaTime;
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
    prevError = error;

    // Ramp-up and ramp-down logic
    float rampUpProgress = constrain((float)elapsedTotal / rampTime, 0.0, 1.0);
    float timeToEnd = moveDuration - elapsedTotal;
    float rampDownProgress = constrain((float)timeToEnd / rampDownDuration, 0.0, 1.0);
    float rampProgress = rampUpProgress * rampDownProgress;

    int currentSpeed = BS + (maxSpeed - BS) * rampProgress;
    int correction = abs(pidOutput) * rampProgress;
    correction = constrain(correction, correctionFactor, maxSpeed);

    Serial.print("Angle: ");
    Serial.print(deg, 2);
    Serial.print(" | PID: ");
    Serial.print(pidOutput);
    Serial.print(" | Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" | Correction: ");
    Serial.println(correction);

if (abs(deg) > deadzone) {
  integral += deg* deltaTime;
  integral = constrain(integral, -maxIntegral, maxIntegral);
} else {
  integral *= 0.95; // Decay integral gradually instead of zeroing
}

    if (abs(deg) < deadzone) {
  integral = 0;
  pidOutput = 0; // Optional: stop micro-corrections
      forward(currentSpeed, currentSpeed);
    } else if (deg >= -deadzone && deg < -10) {
      forward(currentSpeed, currentSpeed - correction);
    } else if (deg <= deadzone && deg > 10) {
      forward(currentSpeed - correction, currentSpeed);
    } 
    //else if (deg >= 10) {
    //   anticlockwise();
    // } else if (deg <= -10) {
    //   clockwise();
    // }
  }
}

void forward(int LS, int RS) {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, LS);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, RS);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, RS);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, LS);
}

void clockwise() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, BS);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, BS);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, BS);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, BS);
  Serial.println("Turning Clockwise");
}

void anticlockwise() {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, BS);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, BS);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, BS);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, BS);
  Serial.println("Turning Anticlockwise");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stop");
}
