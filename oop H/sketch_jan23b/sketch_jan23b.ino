class robot{
  private:
  int motordir[4];
  int motorpwm[4];
  int motorspeed;
  public:
  robot(int FLP, FRP, RRP, RLP, FLD, FRD, RRD, RLD): motorspeed(speed){
    motordir[0] = FLD;
    motordir[1] = FRD;
    motordir[2] = RRD;
    motordir[3] = RLD;;
    motorpwm[0] = FLP;
    motorpwm[1] = FRP;
    motorpwm[2] = RRP;
    motorpwm[3] = RLP;
  }
  void setpins(){
    for(int i = 0; i<4; i++){
      pinMode(motordir[i], OUTPUT);
      pinMode(motorpwm[i], OUTPUT);
    }
  }
  void forward(){
    int dir[4] = {1,1,1,1};
    int speed[4] = {motorspeed, motorspeed, motorspeed, motorspeed};
    setmotors(dir,speed);
    Serial.println("F");
  }
  void back(){
    int dir[4] ={0,0,0,0};
    int speed[4]= {motorspeed, motorspeed, motorspeed, motorspeed};
    setmotors(dir, speed);
    Serial.println("B");
  }
  void right(){
    int dir[4] = {1, 0, 1, 0};
    int speed[4] = { motorspeed, motorspeed, motorspeed, motorspeed};
    setmotors(dir, speed);
    Serial.println("R");
  }
  void left(){
    int dir[4] = {0, 1, 0, 1};
    int speed[4] = {motorspeed, motorspeed, motorspeedm motorspeed};
    Serial.println("L");
  }
  void stop(){
    int speed[4] = {0, 0, 0, 0};
    for(int i = 0; i<4; i++){
      analogWrite(motorpwm[i], speed[i]);
    }
    Serial.println("S");
  }
  private: 
  void setmotors(int direction[], int speed[]){
    for(int i =0; i < 4; i++){
      digitalWrite(motorDir[i], dir[i]);
      analogWrite(motorDir [i], speed[i]);
    }
  }
};
Robot botpins(8, 9, 10, 11, 6, 7, 12, 13, 50);

void setup() {
    botpins.setpins();  
}
void loop() {
forward();
delay(2500);
right();
delay(2500);
reverse();
delay(2500);
left();
delay(2500);
stop();
delay(5000);
}
