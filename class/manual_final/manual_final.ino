#include "ps.h"
#include "drive.h"

PS ps;
Drive drive(14, 30, 15, 31, 18, 32, 19, 33);

#define Conveyer_pwm 22
#define Conveyer_dir 34
#define flywheel_pwm 23
#define flywheel_dir 53

int pwm = 0;
bool lastR1State = false;
bool lastButtonRightState = false;

bool motorOn = false;
bool motorDir = HIGH;

void setup() {
    Serial5.begin(115200);
}

void loop() {
    ps.update();
    conveyer();
    drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
if (ps.l1) {
  Serial.println("L1 is currently PRESSED (digital value 1)");
} else {
  Serial.println("L1 is currently RELEASED (digital value 0)");
}
//flywheel
  if (ps.r1 && !lastR1State) {
    digitalWrite(flywheel_dir, LOW);  // Set direction
    if (pwm == 0) {
      pwm = 150;
    } else {
      pwm += 10;
    }
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheel_pwm, pwm);
    Serial.print("R1 Pressed - PWM: ");
    Serial.println(pwm);
  }
  if (ps.buttonRight && !lastButtonRightState) {
    digitalWrite(flywheel_dir, LOW);  // Still want forward direction
    pwm -= 20;
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheel_pwm, pwm);
    Serial.print("ButtonRight Pressed - PWM: ");
    Serial.println(pwm);
  }

  lastR1State = ps.r1;
  lastButtonRightState = ps.buttonRight;
}
void conveyer() {
    static bool lastButtonUp = false;
    static bool lastButtonDown = false;
    bool upPressed = ps.buttonUp && !lastButtonUp;
    bool downPressed = ps.buttonDown && !lastButtonDown;
    if (upPressed || downPressed) {
        motorDir = upPressed ? HIGH : LOW;
        motorOn = !(motorOn && ((motorDir == HIGH && upPressed) || (motorDir == LOW && downPressed)));
    }
    digitalWrite(Conveyer_dir, motorDir);
    analogWrite(Conveyer_pwm, motorOn ? 255 : 0);
    lastButtonUp = ps.buttonUp;
    lastButtonDown = ps.buttonDown;
}