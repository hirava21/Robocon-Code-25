#include <Servo.h>
Servo motor;

const int motorPin = 24; 
int motorSpeed = 50;     

void setup() {
  motor.attach(motorPin);
  // motor.write(60); 
  // delay(500);
  }
void loop() {
  // int motorspeed=50;
  for (motorSpeed = 0; motorSpeed <= 60; motorSpeed++) {
    // int pwmValue = map(motorSpeed, 0, 100, 0, 60); 
    motor.write(motorSpeed);                         
    delay(50);                                     
  }
  delay(5000);
  for (motorSpeed = 100; motorSpeed >= 0; motorSpeed--) {
    int pwmValue = map(motorSpeed, 0, 100, 0, 70); 
    motor.write(pwmValue);                        
    delay(50);  
  }
  delay(5000);
}