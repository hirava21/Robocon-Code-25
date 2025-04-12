#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int pin1, int pin2, int pin3, int pin4, 
             int pwm1, int pwm2, int pwm3, int pwm4) :
    _pin1(pin1), _pin2(pin2), _pin3(pin3), _pin4(pin4),
    _pwm1(pwm1), _pwm2(pwm2), _pwm3(pwm3), _pwm4(pwm4) {
    // Initialize pins
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin3, OUTPUT);
    pinMode(_pin4, OUTPUT);
    pinMode(_pwm1, OUTPUT);
    pinMode(_pwm2, OUTPUT);
    pinMode(_pwm3, OUTPUT);
    pinMode(_pwm4, OUTPUT);
}

void Motor::drive(bool dir1, bool dir2, bool dir3, bool dir4,
                  int pwm1, int pwm2, int pwm3, int pwm4) {
    // Your existing drive implementation
    digitalWrite(_pin1, dir1);
    digitalWrite(_pin2, dir2);
    digitalWrite(_pin3, dir3);
    digitalWrite(_pin4, dir4);
    analogWrite(_pwm1, pwm1);
    analogWrite(_pwm2, pwm2);
    analogWrite(_pwm3, pwm3);
    analogWrite(_pwm4, pwm4);
}

void Motor::stop() {
    drive(0, 0, 0, 0, 0, 0, 0, 0);
}