#define FL_pwm 14
#define FR_pwm 15
#define RL_pwm 19
#define RR_pwm 18

#define FL_dir 30
#define FR_dir 31
#define RL_dir 33
#define RR_dir 32
int motorSpeed = 50;
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
  forward();
  delay(5000);
  // stop();
  // delay(1000);
  // right();
  // delay(2000);
  // stop();
  // delay(2000);
  // reverse();
  // delay(5000);
  // stop();
  // delay(1000);
  // left();
  // delay(2000);
  // stop();
  // delay(2000);
  // anticlockwise();
  // delay(2000);
  // stop();
  // delay(2000);
  // clockwise();
  // delay(2000);
  // stop();
  // delay(2000);
  // ForwardR45();
  // delay(2000);
  // stop();
  // delay(2000);
  // ReverseL45();
  // delay(2000);
  // stop();
  // delay(2000);
  // ForwardL45();
  // delay(2000);
  // stop();
  // delay(2000);
  // ReverseR45();
  // delay(2000);
  stop();
  delay(5000);
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
void ForwardR45() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, 0);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, 0);
  // Serial.println("anticlockwise");
}
void ForwardL45(){
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, 0);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, 0);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("anticlockwise");
}
void ReverseR45() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, 0);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, 0);
  // Serial.println("anticlockwise");
}
void ReverseL45(){
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, 0);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, 0);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("anticlockwise");
}