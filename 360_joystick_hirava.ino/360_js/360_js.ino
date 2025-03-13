int FLP = 8; int FLD = 9; int FRP = 10; int FRD = 11; int RRP = 12; int RRD = 13; int RLP = 6; int RLD = 7; int max_pwm = 150;
int m1, m2, m3, m4;
int M1, M2, M3, M4;
void setup() {
  Serial.begin(9600); // Monitor
  Serial3.begin(9600); // UART communication with ESP32
  pinMode(FLD, OUTPUT);
  pinMode(FRD, OUTPUT);
  pinMode(RRD, OUTPUT);
  pinMode(RLD, OUTPUT);
  pinMode(FLP, OUTPUT);
  pinMode(FRP, OUTPUT);
  pinMode(RRP, OUTPUT);
  pinMode(RLP, OUTPUT);
}
void loop() {
js();
}
void js(){
    if (Serial3.available() > 0) {
    String data = Serial3.readStringUntil('\n'); // Read joystick, D-pad, throttle, and brake data
    int delimiter1 = data.indexOf(',');
    int delimiter2 = data.indexOf(',', delimiter1 + 1);
    int delimiter3 = data.indexOf(',', delimiter2 + 1);
    int delimiter4 = data.indexOf(',', delimiter3 + 1);

    if (delimiter1 != -1 && delimiter4 != -1) {
      int axisY = data.substring(0, delimiter1).toInt();
      int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
      int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
      int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
      int brake = data.substring(delimiter4 + 1).toInt();

      int uppwm = map(axisY, 0, -512, 0, max_pwm);
      int downpwm = map(axisY, 0, 512, 0, max_pwm);
      int rightpwm = map(axisX,0, -512, 0, max_pwm);
      int leftpwm = map(axisX, 0, 512, 0, max_pwm);

  // Calculate motor PWMs
     int m1pwm = uppwm - rightpwm;
     int m2pwm = uppwm + rightpwm;
     int m3pwm = downpwm - leftpwm;
    int m4pwm = downpwm + leftpwm;

  // Determine motor direction
      m1 = (m1pwm > 0);
      m2 = (m2pwm > 0);
      m3 = (m3pwm < 0);
      m4 = (m4pwm < 0);

  // Limit PWM values
      M1 = constrain(abs(m1pwm), 0, max_pwm);
      M2 = constrain(abs(m2pwm), 0, max_pwm);
      M3 = constrain(abs(m3pwm), 0, max_pwm);
      M4 = constrain(abs(m4pwm), 0, max_pwm);

digitalWrite(FLD, m1);
digitalWrite(FRD, m2);
digitalWrite(RRD, m3);
digitalWrite(RLD, m4);
analogWrite(FLP, M1);
analogWrite(FRP, M2);
analogWrite(RRP, M3);
analogWrite(RLP, M4);
     }
    }
  }

