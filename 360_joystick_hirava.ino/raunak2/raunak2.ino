#include <math.h>

// Motor pins
int PWM1 = 8, DIR1 = 9;
int PWM2 = 10, DIR2 = 11;
int PWM3 = 12, DIR3 = 13;
int PWM4 = 6, DIR4 = 7;

// Constants
const int max_pwm = 110;
const int rot_pwm = 50;
const int deadZone = 200;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);  // Monitor
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
    String data = Serial3.readStringUntil('\n'); // Read joystick data
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

      // Convert joystick X/Y to angle and magnitude
      float X = axisX / 512.0;
      float Y = -axisY / 512.0; // Negative because joystick Y is reversed
      float angle = atan2(Y, X) * 180.0 / PI;
      if (angle < 0) angle += 360;

      float magnitude = sqrt(X * X + Y * Y);
      motorSpeed = map(magnitude * 512, 0, 512, 0, max_pwm);

      // Handling throttle and brake for rotation
      int clkpwm = (throttle > 0) ? rot_pwm : 0;
      int antclkpwm = (brake > 0) ? rot_pwm : 0;

      // Convert joystick values to motor control
      int uppwm = map(axisY, 0, -512, 0, max_pwm);
      int downpwm = map(axisY, 0, 512, 0, max_pwm);
      int rightpwm = map(axisX, 0, 512, 0, max_pwm);
      int leftpwm = map(axisX, 0, -512, 0, max_pwm);

      int m1pwm = uppwm - rightpwm + clkpwm - antclkpwm; // Front Left
      int m2pwm = uppwm + rightpwm - clkpwm + antclkpwm; // Front Right
      int m3pwm = downpwm - leftpwm + clkpwm - antclkpwm; // Rear Right
      int m4pwm = downpwm + leftpwm - clkpwm + antclkpwm; // Rear Left

      // Determine motor direction
      bool m1 = (m1pwm > 0);
      bool m2 = (m2pwm > 0);
      bool m3 = (m3pwm < 0);
      bool m4 = (m4pwm < 0);

      // Limit PWM values
      int M1 = constrain(abs(m1pwm), 0, max_pwm);
      int M2 = constrain(abs(m2pwm), 0, max_pwm);
      int M3 = constrain(abs(m3pwm), 0, max_pwm);
      int M4 = constrain(abs(m4pwm), 0, max_pwm);

      // Move motors
      moveMotor(PWM1, DIR1, M1, m1);
      moveMotor(PWM2, DIR2, M2, m2);
      moveMotor(PWM3, DIR3, M3, m3);
      moveMotor(PWM4, DIR4, M4, m4);

      // Stop condition
      if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
        stop();
        Serial.println("STOP");
      }
    }
  }
}

// Function to move motors
void moveMotor(int pwmPin, int dirPin, int speed, bool direction) {
  digitalWrite(dirPin, direction);
  analogWrite(pwmPin, speed);
}

// Stop all motors
void stop() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
}
