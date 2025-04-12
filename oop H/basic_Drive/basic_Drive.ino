class robot {
  private:
    int motordir[4];
    int motorpwm[4];
    int motorspeed;
  public:
    robot(int FLP, int FRP, int RRP, int RLP, int FLD, int FRD, int RRD, int RLD, int speed)
        : motorspeed(speed) {
      motordir[0] = FLD;
      motordir[1] = FRD;
      motordir[2] = RRD;
      motordir[3] = RLD;
      motorpwm[0] = FLP;
      motorpwm[1] = FRP;
      motorpwm[2] = RRP;
      motorpwm[3] = RLP;
    }

    void setpins() {
      for (int i = 0; i < 4; i++) {
        pinMode(motordir[i], OUTPUT);
        pinMode(motorpwm[i], OUTPUT);
      }
    }

    void forward() {
      int dir[4] = {1, 1, 1, 1};
      int speed[4] = {motorspeed, motorspeed, motorspeed, motorspeed};
      setmotors(dir, speed);
      Serial.println("F");
    }

    void back() {
      int dir[4] = {0, 0, 0, 0};
      int speed[4] = {motorspeed, motorspeed, motorspeed, motorspeed};
      setmotors(dir, speed);
      Serial.println("B");
    }

    void right() {
      int dir[4] = {1, 0, 1, 0};
      int speed[4] = {motorspeed, motorspeed, motorspeed, motorspeed};
      setmotors(dir, speed);
      Serial.println("R");
    }

    void left() {
      int dir[4] = {0, 1, 0, 1};
      int speed[4] = {motorspeed, motorspeed, motorspeed, motorspeed};
      setmotors(dir, speed);
      Serial.println("L");
    }

    void stop() {
      int speed[4] = {0, 0, 0, 0};
      for (int i = 0; i < 4; i++) {
        analogWrite(motorpwm[i], speed[i]);
      }
      Serial.println("S");
    }

  private:
    void setmotors(int dir[], int speed[]) {
      for (int i = 0; i < 4; i++) {
        digitalWrite(motordir[i], dir[i]);
        analogWrite(motorpwm[i], speed[i]);
      }
    }
};
robot botpins(8, 10, 12, 6, 9, 11, 13, 7, 50);

void setup() {
  Serial.begin(9600); 
  botpins.setpins();
}

void loop() {
  botpins.forward();
  delay(2500);
  botpins.right();
  delay(2500);
  botpins.back(); 
  delay(2500);
  botpins.left();
  delay(2500);
  botpins.stop();
  delay(5000);
}
