#include <SoftwareSerial.h>

SoftwareSerial Xbee(2, 3); // RX, TX

// Pin definitions
const int xPin = A4;       // X-axis for movement
const int yPin = A1;       // Y-axis for movement
const int cw_pin = 5;      // Clockwise pin
const int acw_pin = 6;     // Anticlockwise pin

const int max_pwm = 110;
const int rot_pwm = 50;
const int deadZone = 200;

// Variables
int clkpwm, antclkpwm;
int M1, M2, M3, M4;
bool m1, m2, m3, m4;

void setup() {
  Serial.begin(9600);
  Xbee.begin(9600);
  pinMode(cw_pin, INPUT);
  pinMode(acw_pin, INPUT);
}

void loop() {
  // Read joystick inputs
  int xVal = analogRead(xPin);  
  int yVal = analogRead(yPin);

  Serial.print("xVal ");
  Serial.print(xVal);
  Serial.print("  yVal ");
  Serial.println(yVal);

  // Determine clockwise and counterclockwise rotation
  clkpwm = (digitalRead(cw_pin) > 0) ? rot_pwm : 0;
  antclkpwm = (digitalRead(acw_pin) > 0) ? rot_pwm : 0;

  // Calculate PWM signals for each motor based on joystick input
  int uppwm = map(xVal, 506, 1023, 0, max_pwm);
  int downpwm = map(xVal, 505, 0, 0, max_pwm);
  int rightpwm = map(yVal, 502, 1023, 0, max_pwm);
  int leftpwm = map(yVal, 501, 0, 0, max_pwm);

  // Calculate motor PWMs
  int m1pwm = uppwm - rightpwm + clkpwm - antclkpwm; // Motor 1 (FL)
  int m2pwm = uppwm + rightpwm - clkpwm + antclkpwm; // Motor 2 (FR)
  int m3pwm = downpwm - leftpwm + clkpwm - antclkpwm; // Motor 3 (RR)
  int m4pwm = downpwm + leftpwm - clkpwm + antclkpwm; // Motor 4 (RL)

  // Determine motor direction
  m1 = (m1pwm > 0); //dir
  m2 = (m2pwm > 0);
  m3 = (m3pwm < 0);
  m4 = (m4pwm < 0);

  // Limit PWM values
  M1 = constrain(abs(m1pwm), 0, max_pwm); //pwm
  M2 = constrain(abs(m2pwm), 0, max_pwm);
  M3 = constrain(abs(m3pwm), 0, max_pwm);
  M4 = constrain(abs(m4pwm), 0, max_pwm);

  // Prepare and send data
  String dataToSend = String(M1) + "," + String(M2) + "," + String(M3) + "," +
                      String(M4) + "," + String(m1) + "," + String(m2) + "," +
                      String(m3) + "," + String(m4);
  
  // Serial.println("Data to send: " + dataToSend);
  Xbee.println(dataToSend);

  delay(50); // Adjust delay as needed to avoid buffer overflow and ensure stability
}
