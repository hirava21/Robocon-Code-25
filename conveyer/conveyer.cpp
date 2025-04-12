#include "conveyer.h"

Conveyer::Conveyer(int pwm, int dir) : PWM(pwm), DIR(dir) {}

void Conveyer::begin() {
    pinMode(PWM, OUTPUT);
    pinMode(DIR, OUTPUT);
    stop();  // Start in stopped state
}

void Conveyer::up() {
    digitalWrite(DIR, HIGH);   // Set direction forward
    analogWrite(PWM, 255);     // Full speed
}

void Conveyer::down() {
    digitalWrite(DIR, LOW);    // Set direction reverse
    analogWrite(PWM, 255);     // Full speed
}

void Conveyer::stop() {
    analogWrite(PWM, 0);       // Stop motor
}
