#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    // Your existing motor pin definitions
    int _pin1, _pin2, _pin3, _pin4;
    int _pwm1, _pwm2, _pwm3, _pwm4;
    
public:
    Motor(int pin1, int pin2, int pin3, int pin4, 
          int pwm1, int pwm2, int pwm3, int pwm4);
    void drive(bool dir1, bool dir2, bool dir3, bool dir4,
               int pwm1, int pwm2, int pwm3, int pwm4);
    void stop();  // New method
};

#endif