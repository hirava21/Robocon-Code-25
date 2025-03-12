#include <PID_v1.h>
#include <SparkFun_BNO080_Arduino_Library.h>
BNO080 myIMU;
int PWM1 = 8, DIR1 = 9;  int PWM2 = 10, DIR2 = 11;  
int PWM3 = 12, DIR3 = 13;int PWM4 = 6, DIR4 = 7;  
int motorSpeed = 0;
float initialYaw = 0.0;
float adjustedYaw = 0.0;
float target_angle = 0.0;

// PID variables
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.1, Kd = 0.05;  // Tune these values based on performance
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);
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

  // Calculate initial yaw
  float sumYaw = 0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
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
    pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
    pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);
    

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-150, 150);  // Limit motor speed changes
}

void loop() {
    js();
    if (myIMU.dataAvailable()) {
        float qw = myIMU.getQuatReal();
        float qx = myIMU.getQuatI();
        float qy = myIMU.getQuatJ();
        float qz = myIMU.getQuatK();
        float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        yaw = yaw * 180.0 / PI;
        adjustedYaw = yaw - initialYaw;
        if (adjustedYaw > 180) adjustedYaw -= 360;
        else if (adjustedYaw < -180) adjustedYaw += 360;
        Serial.print("Adjusted Yaw: ");
        Serial.println(adjustedYaw, 2);
    }
    pidControl();
    delay(100);
}

float calculateTargetAngle(int throttle, int brake, float max_angular_speed) {
    static unsigned long last_update_time = 0;
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0;
    if (dt > 0.0) {
        last_update_time = current_time;
        float omega = ((throttle - brake) / 1024.0) * max_angular_speed;
        target_angle = fmod((target_angle + (omega * dt)), 360.0);
        if (target_angle < 0) target_angle += 360.0;
    }
    return target_angle;
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
   int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
   int brake = data.substring(delimiter4 + 1).toInt();
   target_angle = calculateTargetAngle(throttle, brake, 180.0);
   Serial.print("Target Angle: ");
   Serial.println(target_angle);

        if (axisY <= -25) {
         motorSpeed = map(axisY, -25, -512, 0, 150);
         forward();
         Serial.println("Forward");
            } else if (axisY >= 25) {
                motorSpeed = map(axisY, 25, 512, 0, 150);
                reverse();
                Serial.println("Reverse");
            }

            if (axisX <= -25) {
                motorSpeed = map(axisX, -25, -512, 0, 150);
                left();
                Serial.println("Left");
            } else if (axisX >= 25) {
                motorSpeed = map(axisX, 25, 512, 0, 150);
                right();
                Serial.println("Right");
            }

            if (throttle > 0) {
                motorSpeed = map(throttle, 0, 1024, 0, 150);
                clockwise();
                Serial.println("Clockwise");
            }

            if (brake > 0) {
                motorSpeed = map(brake, 0, 1024, 0, 150);
                anticlockwise();
                Serial.println("Anticlockwise");
            }

            if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
                stop();
                Serial.println("STOP");
            }
        }
    }
}

void pidControl() {
    Input = adjustedYaw;
    Setpoint = target_angle;
    myPID.Compute();
    motorSpeed = constrain(abs(Output), 0, 150);
    if (Output > 0) forward();
    else if (Output < 0) reverse();
    else stop();
}

void forward() {
    digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}

void reverse() {
    digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}

void stop() {
    analogWrite(PWM1, 0); analogWrite(PWM2, 0);
    analogWrite(PWM3, 0); analogWrite(PWM4, 0);
}
void left() {
    digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void right() {
    digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void clockwise() {
    digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void anticlockwise() {
    digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
