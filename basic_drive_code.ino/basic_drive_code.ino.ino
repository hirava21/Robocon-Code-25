int FL_pwm = 5; int FR_pwm = 9; int RL_pwm = 11; RR_pwm =3;
int FL_dir = 6; int FR_dir = 10; int RL_dir = 12; RR_dir =4;
void setup(){
  pinMode(FL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT);
}

void forward(){
digitalWrite(FL_dir,1);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,1);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,1);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,1);
analogWrite(RL_pwm, motorSpeed);
Serial.println("forward");
}
void reverse(){
digitalWrite(FL_dir,0);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,0);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,0);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,0);
analogWrite(RL_pwm, motorSpeed);
Serial.println("reverse");
}
void right(){
digitalWrite(FL_dir,1);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,0);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,1);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,0);
analogWrite(RL_pwm, motorSpeed);
Serial.println("right");
}
void left(){
digitalWrite(FL_dir,0);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,1);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,0);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,1);
analogWrite(RL_pwm, motorSpeed);
Serial.println("left");
}
void stop(){
digitalWrite(FL_dir,0);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,0);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,0);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,0);
analogWrite(RL_pwm, motorSpeed);
Serial.println("stop");
}
void clockwise() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("clockwise");
}
void anticlockwise() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("anticlockwise");
}




