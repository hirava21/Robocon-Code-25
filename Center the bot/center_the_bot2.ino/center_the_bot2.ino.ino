#include <SoftwareSerial.h>
SoftwareSerial mySerial1(69, 68); 
SoftwareSerial mySerial2(66, 67);
int FL_pwm = 5; int FR_pwm = 9; int RL_pwm = 11; int RR_pwm = 3; int TCL = 0; int TCU = 0;
int FL_dir = 6; int FR_dir = 10; int RL_dir = 12; int RR_dir = 4; int motorSpeed = 255;
int TFL = 0;  int TFU = 0;  

void setup() {
  Serial.begin(115200);
  mySerial2.begin(115200);
  mySerial1.begin(115200);

  pinMode(FL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT);
}
void loop() {
  getval();
  getinC();
}
void getTFminiData2(int* distance, boolean* complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (mySerial2.available()) {
    rx[i] = mySerial2.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
void getTFminiData1(int* distance, boolean* complete) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (mySerial1.available()) {
    rx[i] = mySerial1.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
void getval() {
  boolean receiveComplete1 = false;
  boolean receiveComplete2 = false;

  getTFminiData2(&TFL, &receiveComplete2);
  if (receiveComplete2) {
    Serial.print("TFL==");
    Serial.print(TFL);
    Serial.print("cm\t");
  }
  getTFminiData1(&TFU, &receiveComplete1);
  if (receiveComplete1) {
    Serial.print("TFU=");
    Serial.print(TFU);
    Serial.print("cm\t");
  }
}
void getinC() {
  if (TFL > 90) {
    left();
  } else if (TFL < 90) {
    right();
  } else {
    if (TFU > 220) {
      forward();
    } else if (TFU < 220) {
      reverse();
    } else {
      stop();
    }
  }
}
void forward() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("forward");
}
void reverse() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("reverse");
}
void right() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("right");
}
void left() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("left");
}
void stop() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, 0);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, 0);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, 0);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("stop");
}
