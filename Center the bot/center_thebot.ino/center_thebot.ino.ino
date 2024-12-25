#include <TFMini.h>
TFMini tfminiLeft;  TFMini tfminiRight; 
#define FL_pwm  5
#define FL_dir  6
#define FR_pwm  9
#define FR_dir  10
#define RL_pwm  3
#define RL_dir  4
#define RR_pwm  11
#define RR_dir  12
int speedFL, speedFR, speedRL, speedRR; 
void setup() {
  Serial.begin(9600); // Arduino serial monitor
  Serial1.begin(115200); tfminiLeft.begin((Stream*)&Serial1); 
  Serial2.begin(115200); tfminiRight.begin((Stream*)&Serial2); 
  pinMode(FL_dir, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RL_dir, OUTPUT);
}

void setspeed(int pwmPin, int dirPin, int speed) {
  if (speed > 0) {
    digitalWrite(dirPin, HIGH); 
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);  
    analogWrite(pwmPin, -speed);
  }
}
void algo() {
  uint16_t LD = tfminiLeft.getDistance(); 
  uint16_t RD = tfminiRight.getDistance(); 
  if (LD != 0 && RD != 0) { 
    int centerError = LD - RD; 
    int forwardSpeed = 220; 
    speedFL = forwardSpeed + centerError;
    speedFR = forwardSpeed - centerError;
    speedRL = forwardSpeed + centerError;
    speedRR = forwardSpeed - centerError;

    speedFL = constrain(speedFL, 0, 255);
    speedFR = constrain(speedFR, 0, 255);
    speedRL = constrain(speedRL, 0, 255);
    speedRR = constrain(speedRR, 0, 255);

    Serial.print("LD: ");
    Serial.print(LD);
    Serial.print(", RD: ");
    Serial.print(RD);
    Serial.print(", Error: ");
    Serial.println(centerError);
  } else {
    setspeed(FL_pwm, FL_dir, 0);
    setspeed(FR_pwm, FR_dir, 0);
    setspeed(RR_pwm, RR_dir, 0);
    setspeed(RL_pwm, RL_dir, 0);
    Serial.println("Invalid sensor reading");
  }
}
void loop() {
  algo(); 
  setspeed(FL_pwm, FL_dir, speedFL);
  setspeed(FR_pwm, FR_dir, speedFR);
  setspeed(RR_pwm, RR_dir, speedRR);
  setspeed(RL_pwm, RL_dir, speedRL);    
  delay(50);
}
