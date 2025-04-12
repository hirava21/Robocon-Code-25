int PWM1 = 8; int DIR1 = 9; int PWM2 = 10; int DIR2 = 11; int PWM3 = 12; int DIR3 = 13;int PWM4 = 6;int DIR4 = 7; int motorSpeed = 0;
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
BNO080 myIMU;
float initialYaw = 0; float adjustedYaw = 0; int BS = 100; int correctionFactor =60;
void setup() {
  Serial.begin(9600); // Monitor
  Serial3.begin(9600); // UART communication with ESP32
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("Starting setup...");
  Wire.begin();
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO085 not detected. Check your wiring or I2C address.");
    while (1); 
  }
  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);
  delay(1000);
  float sumYaw = 0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable() == true) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();
      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;
      sumYaw += yaw;
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
  pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);
}
void loop() {
js();
}
void js(){
    if (Serial3.available() > 0) {
    String data = Serial3.readStringUntil('\n'); // Read joystick, D-pad, throttle, and brake data
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
      if (axisY <= -25) {
        motorSpeed = map(axisY, -25, -512, 0, 50);
        Forward();
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
    }
  }
}
void Forward(){
  if (myIMU.dataAvailable() == true) {
    // Read the quaternion data
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    float adjustedYaw = yaw - initialYaw;
    if (adjustedYaw > 180) {
      adjustedYaw -= 360;
    } else if (adjustedYaw < -180) {
      adjustedYaw += 360;
    }
    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);
  if (adjustedYaw > -5 && adjustedYaw < 5) {
    Serial.println("Moving Straight");
    forward(BS, BS);
  } 
  else if (adjustedYaw >= 5 && adjustedYaw < 10) {
    Serial.println("Correcting Right (Slow down Right)");
    forward(BS, BS - correctionFactor);
  } 
  else if (adjustedYaw <= -5 && adjustedYaw > -10) {
    Serial.println("Correcting Left (Slow down Left)");
    forward(BS - correctionFactor, BS);
  } 
  else if (adjustedYaw >= 10) {
    Serial.println("Turning Clockwise");
    clockwise();
  } 
  else if (adjustedYaw <= -10) {
    Serial.println("Turning Anticlockwise");
    anticlockwise();
  }
}
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
void forward(int LS, int RS) {
  digitalWrite(DIR1, 1); analogWrite(PWM1, LS);
  digitalWrite(DIR2, 1); analogWrite(PWM2, RS);
  digitalWrite(DIR3, 1); analogWrite(PWM3, RS);
  digitalWrite(DIR4, 1); analogWrite(PWM4, LS);
}