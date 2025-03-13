#include <math.h>

int PWM1 = 8; int DIR1 = 9;
int PWM2 = 10; int DIR2 = 11;
int PWM3 = 12; int DIR3 = 13;
int PWM4 = 6; int DIR4 = 7;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600); // Monitor
  Serial3.begin(9600); // UART communication with ESP32

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
}

void js() {
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

      // Convert Joystick X/Y to angle and magnitude
      float X = axisX / 512.0;
      float Y = -axisY / 512.0; // Negative because joystick Y is reversed
      float angle = atan2(Y, X) * 180.0 / PI;
      if (angle < 0) angle += 360; // Convert to 0-360 range

      float magnitude = sqrt(X * X + Y * Y);
      motorSpeed = map(magnitude * 512, 0, 512, 0, 50); // Mapping speed as per joystick intensity

      // Handling special cases: throttle & brake
      if (throttle > 0) {
        motorSpeed = map(throttle, 0, 1024, 0, 50);
        clockwise();
        Serial.println("Clockwise");
      } else if (brake > 0) {
        motorSpeed = map(brake, 0, 1024, 0, 50);
        anticlockwise();
        Serial.println("Anticlockwise");
      } 
      // General 360-degree movement based on angle
      else {
        moveRobot(angle);
      }
      
      // Stop condition
      if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
        stop();
        Serial.println("STOP");
      }
    }
  }
}

// Function to move in any direction based on angle
void moveRobot(float angle) {
  if (angle >= 337.5 || angle < 22.5) {
    forward();
    Serial.println("Forward");
  } else if (angle >= 22.5 && angle < 67.5) {
    forwardRight();
    Serial.println("Forward-Right");
  } else if (angle >= 67.5 && angle < 112.5) {
    right();
    Serial.println("Right");
  } else if (angle >= 112.5 && angle < 157.5) {
    reverseRight();
    Serial.println("Reverse-Right");
  } else if (angle >= 157.5 && angle < 202.5) {
    reverse();
    Serial.println("Reverse");
  } else if (angle >= 202.5 && angle < 247.5) {
    reverseLeft();
    Serial.println("Reverse-Left");
  } else if (angle >= 247.5 && angle < 292.5) {
    left();
    Serial.println("Left");
  } else if (angle >= 292.5 && angle < 337.5) {
    forwardLeft();
    Serial.println("Forward-Left");
  }
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

void forwardLeft() {
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed / 2);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed / 2);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}

void forwardRight() {
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed / 2);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed / 2);
}

void reverseLeft() {
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed / 2);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed / 2);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}

void reverseRight() {
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed / 2);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed / 2);
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

void stop() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
}
