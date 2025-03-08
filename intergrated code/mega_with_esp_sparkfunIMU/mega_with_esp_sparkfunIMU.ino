#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
BNO080 myIMU;
int PWM1 = 8; int DIR1 = 9; int PWM2 = 10; int DIR2 = 11; int PWM3 = 12; int DIR3 = 13;int PWM4 = 6;int DIR4 = 7; int motorSpeed = 0;
float initialYaw = 0, adjustedYaw = 0;
float targetedYaw = 180;  // Target yaw in degrees
bool hasReachedTarget = false;  // Flag to track target reach
void setup() {
  Serial.begin(9600); // Monitor
  Serial3.begin(9600); // UART communication with ESP32
  Serial.begin(115200);
    while (!Serial);
    Serial.println("Starting setup...");
    Wire.begin();
    if (!myIMU.begin(0x4A)) {
        Serial.println("BNO085 not detected. Check wiring or I2C address.");
        while (1);
    }
    Serial.println("BNO085 detected!");
    myIMU.enableRotationVector(100);  // Update every 100ms
    delay(1000);
    float sumYaw = 0;
    int validReadings = 0;
    for (int i = 0; i < 50; i++) {  // Reduced to 50 iterations for efficiency
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
    Serial.println("Setup complete, starting loop...");
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
js();
  if (myIMU.dataAvailable()) {
   float currentYaw = getYaw();
   adjustedYaw = currentYaw - initialYaw;
   if (adjustedYaw < 0) adjustedYaw += 360;
   if (adjustedYaw > 360) adjustedYaw -= 360;
  //  Serial.print("Adjusted Yaw: ");
  //  Serial.println(adjustedYaw, 2);
   }
  }
void js(){
  if (Serial3.available() > 0) {
    String data = Serial3.readStringUntil('\n'); // Read joystick, D-pad, throttle, and brake data
    int delimiter1 = data.indexOf(',');
    int delimiter2 = data.indexOf(',', delimiter1 + 1);
    int delimiter3 = data.indexOf(',', delimiter2 + 1);
    int delimiter4 = data.indexOf(',', delimiter3 + 1);
    int delimiter5 = data.indexOf(',', delimiter4 + 1);

    if (delimiter1 != -1 && delimiter5 != -1) {
      int axisY = data.substring(0, delimiter1).toInt();
      int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
      int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
      int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
      int brake = data.substring(delimiter4 + 1, delimiter5).toInt();
      int buttons = data.substring(delimiter5 + 1).toInt();

      if (axisY <= -25) {
        motorSpeed = map(axisY, -25, -512, 0, 50);
        forward();
        Serial.println("Forward");
      } else if (axisY >= 25) {
        motorSpeed = map(axisY, 25, 512, 0, 50);
        reverse();
        Serial.println("Reverse");
      }
      if (axisX <= -25) {
        motorSpeed = map(axisX, -25, -512, 0,50);
        left();
        Serial.println("Left");
      } else if (axisX >= 25) {
        motorSpeed = map(axisX, 25, 512, 0, 50);
        right();
        Serial.println("Right");
      }
      if (throttle > 0) {
        motorSpeed = map(throttle, 0,1024, 0, 50);
        clockwise();
        Serial.println("Clockwise");
      }
      if (brake > 0) {
        motorSpeed = map(brake, 0,1024, 0, 50);
        anticlockwise();
        Serial.println("Antivclck");
      }
      if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
          stop();
        Serial.println("STOP");
      }
      if (buttons == 2){
      float error = targetedYaw - adjustedYaw;
      if (error > 180) error -= 360;
      if (error < -180) error += 360;
      if (abs(error) <= 1) {
      stop();
      Serial.println("Targeted angle reached!");
      hasReachedTarget = true;
      return;
    }

    // Rotate in the shortest direction
    if (targetedYaw > 180) {
        clockwise();
    } else {
        anticlockwise();
    }
      }
    }
  }
}
void forward(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void reverse(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void left(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void right(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH);analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void clockwise(){
  digitalWrite(DIR1, HIGH);analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void anticlockwise(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void stop(){
  analogWrite(PWM1, 0);analogWrite(PWM2, 0);
  analogWrite(PWM3, 0); analogWrite(PWM4, 0);
}
float getYaw() {
  float qw = myIMU.getQuatReal();
  float qx = myIMU.getQuatI();
  float qy = myIMU.getQuatJ();
  float qz = myIMU.getQuatK();
  float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  yaw = yaw * 180.0 / PI;  // Convert to degrees
    return yaw;
}