#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;  

// Motor Pins
int PWM1 = 8, DIR1 = 9;   
int PWM2 = 10, DIR2 = 11;  
int PWM3 = 12, DIR3 = 13;  
int PWM4 = 6, DIR4 = 7;  

int motorSpeed = 50;  // Default speed

// IMU Parameters
float initialYaw = 0, targetYaw = 0;
float kp_yaw = 0.99, kd_yaw = 0.5;
float lastYawError = 0, YawCorrection = 0;

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);  // Communication with ESP32
    Wire.begin();
    
    // IMU Initialization
    if (!myIMU.begin(0x4A)) {
        Serial.println("BNO085 not detected. Check wiring or I2C address.");
        while (1);
    }
    Serial.println("BNO085 detected!");
    myIMU.enableRotationVector(100);
    
    // Calculate Initial Yaw
    float sumYaw = 0;
    int validReadings = 0;
    for (int i = 0; i < 50; i++) { 
        if (myIMU.dataAvailable()) {
            sumYaw += getYaw();
            validReadings++;
        }
        delay(10);
    }
    if (validReadings > 0) {
        initialYaw = sumYaw / validReadings;
        Serial.print("Initial Yaw: ");
        Serial.println(initialYaw, 2);
    } else {
        Serial.println("Failed to get initial yaw.");
    }

    targetYaw = initialYaw;  // Set the initial yaw as the target

    // Motor Pin Modes
    pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
    pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);
}

void loop() {
    js();  // Read joystick inputs
}

void js() {
  if (Serial3.available() > 0) {
  String data = Serial3.readStringUntil('\n');
  int delimiter1 = data.indexOf(',');
  int delimiter2 = data.indexOf(',', delimiter1 + 1);
  int delimiter3 = data.indexOf(',', delimiter2 + 1);
  int delimiter4 = data.indexOf(',', delimiter3 + 1);
 if (delimiter1 != -1 && delimiter4 != -1) {
 int axisY = data.substring(0, delimiter1).toInt();
 int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
 int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
  int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
  int brake = data.substring(delimiter4 + 1).toInt();

  if (myIMU.dataAvailable()) {
  float currentYaw = getYaw();
  float yawError = targetYaw - currentYaw;
  YawCorrection = kp_yaw * yawError + kd_yaw * (yawError - lastYawError);
  lastYawError = yawError;
  }
  if (axisY <= -25) {
   motorSpeed = map(axisY, -25, -512, 0, 50);
   moveForward(YawCorrection);
   Serial.println("Forward");
  } else if (axisY >= 25) {
  motorSpeed = map(axisY, 25, 512, 0, 50);
  moveReverse(YawCorrection);
  Serial.println("Reverse");
}
 if (axisX <= -25) {
 motorSpeed = map(axisX, -25, -512, 0, 50);
 moveLeft(YawCorrection);
 Serial.println("Left");
  } else if (axisX >= 25) {
 motorSpeed = map(axisX, 25, 512, 0, 50);
 moveRight(YawCorrection);
 Serial.println("Right");
 }
 if (throttle > 0) {
 motorSpeed = map(throttle, 0, 1024, 0, 50);
 rotateClockwise();
 Serial.println("Clockwise");
 }
 if (brake > 0) {
  motorSpeed = map(brake, 0, 1024, 0, 50);
  rotateAnticlockwise();
  Serial.println("AntiClockwise");
  }
  if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
    stopMotors();
    Serial.println("STOP");
    }
       }
    }
}

// Yaw-corrected movement functions
void moveForward(float yawCorrection) {
    int speedL = motorSpeed - yawCorrection;
    int speedR = motorSpeed + yawCorrection;

    analogWrite(PWM1, speedL); digitalWrite(DIR1, HIGH);
    analogWrite(PWM2, speedR); digitalWrite(DIR2, HIGH);
    analogWrite(PWM3, speedL); digitalWrite(DIR3, HIGH);
    analogWrite(PWM4, speedR); digitalWrite(DIR4, HIGH);
}

void moveReverse(float yawCorrection) {
    int speedL = motorSpeed - yawCorrection;
    int speedR = motorSpeed + yawCorrection;

    analogWrite(PWM1, speedL); digitalWrite(DIR1, LOW);
    analogWrite(PWM2, speedR); digitalWrite(DIR2, LOW);
    analogWrite(PWM3, speedL); digitalWrite(DIR3, LOW);
    analogWrite(PWM4, speedR); digitalWrite(DIR4, LOW);
}

void moveLeft(float yawCorrection) {
    analogWrite(PWM1, motorSpeed - yawCorrection); digitalWrite(DIR1, LOW);
    analogWrite(PWM2, motorSpeed + yawCorrection); digitalWrite(DIR2, HIGH);
    analogWrite(PWM3, motorSpeed - yawCorrection); digitalWrite(DIR3, LOW);
    analogWrite(PWM4, motorSpeed + yawCorrection); digitalWrite(DIR4, HIGH);
}

void moveRight(float yawCorrection) {
    analogWrite(PWM1, motorSpeed + yawCorrection); digitalWrite(DIR1, HIGH);
    analogWrite(PWM2, motorSpeed - yawCorrection); digitalWrite(DIR2, LOW);
    analogWrite(PWM3, motorSpeed + yawCorrection); digitalWrite(DIR3, HIGH);
    analogWrite(PWM4, motorSpeed - yawCorrection); digitalWrite(DIR4, LOW);
}

void rotateClockwise() {
    analogWrite(PWM1, motorSpeed); digitalWrite(DIR1, HIGH);
    analogWrite(PWM2, motorSpeed); digitalWrite(DIR2, LOW);
    analogWrite(PWM3, motorSpeed); digitalWrite(DIR3, LOW);
    analogWrite(PWM4, motorSpeed); digitalWrite(DIR4, HIGH);
}

void rotateAnticlockwise() {
    analogWrite(PWM1, motorSpeed); digitalWrite(DIR1, LOW);
    analogWrite(PWM2, motorSpeed); digitalWrite(DIR2, HIGH);
    analogWrite(PWM3, motorSpeed); digitalWrite(DIR3, HIGH);
    analogWrite(PWM4, motorSpeed); digitalWrite(DIR4, LOW);
}

void stopMotors() {
    analogWrite(PWM1, 0); analogWrite(PWM2, 0);
    analogWrite(PWM3, 0); analogWrite(PWM4, 0);
}

// Function to get current yaw angle
float getYaw() {
    float yaw = atan2(2.0 * (myIMU.getQuatReal() * myIMU.getQuatK() + myIMU.getQuatI() * myIMU.getQuatJ()), 
                      1.0 - 2.0 * (myIMU.getQuatJ() * myIMU.getQuatJ() + myIMU.getQuatK() * myIMU.getQuatK()));
    return yaw * 180.0 / PI;
}

// void forward(){
//   analogWrite(m1, speed_FL); digitalWrite(M1, 1);
//     analogWrite(m2, speed_FR); digitalWrite(M2, 1);
//     analogWrite(m3, speed_BL); digitalWrite(M3, 1);
//     analogWrite(m4, speed_BR); digitalWrite(M4, 1);
// }