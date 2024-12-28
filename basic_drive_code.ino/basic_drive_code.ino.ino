int FL_pwm = 2; int FR_pwm = 4; int RL_pwm = 8; int RR_pwm =6;
int FL_dir = 3; int FR_dir = 5; int RL_dir = 9; int RR_dir =7; int motorSpeed = 100;
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
void loop(){
  // forward();
  // delay(5000);
  // stop();
  // delay(2000);
  // right();
  // delay(5000);
  // stop();
  // delay(2000);
  // reverse();
  // delay(5000);
  // stop();
  // delay(2000);
  // left();
  // delay(5000);
  // stop();
  // delay(2000);
  anticlockwise();
  delay(5000);
  stop();
  delay(2000);
  clockwise();
  delay(5000);
  stop();
  delay(2000);
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
analogWrite(FL_pwm, 0);
analogWrite(FR_pwm, 0);
analogWrite(RR_pwm, 0);
analogWrite(RL_pwm, 0);
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
