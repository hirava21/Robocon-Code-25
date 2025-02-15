int PWM1 = 8;
int DIR1 = 9;
int PWM2 = 10;
int DIR2 = 11;
int PWM3 = 12;
int DIR3 = 13;
int PWM4 = 6;
int DIR4 = 7;
int motorSpeed = 30;

volatile unsigned int counter1 = 0;  // Counter for encoder 1 pulses
volatile unsigned int counter2 = 0;  // Counter for encoder 2 pulses

unsigned long lastTime = 0;          // To store the last time value was sent
const long interval = 1000;          // Interval at which to send results (1000 ms = 1 second)
const int pulsesPerRevolution = 600; // Encoder's pulses per revolution (PPR)
const float wheelDiameter = 10;      // Diameter of the wheel in cm
const float wheelCircumference = 3.14159 * wheelDiameter; // Circumference of the wheel in cm

float totalDistance1 = 0.0;          // Total distance traveled for encoder 1 in cm
float totalDistance2 = 0.0;          // Total distance traveled for encoder 2 in cm
float distanceThisInterval1 = 0.0;
float distanceThisInterval2 = 0.0;

bool moveForwardFlag = false;  // Flag to control forward movement
bool moveRightFlag = false;    // Flag to control right movemen

void setup() {
  Serial.begin(9600); // Monitor
  Serial3.begin(9600); // UART communication with ESP32

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  // Configure encoder pins as input
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  // Enable pull-up resistors
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

  // Attach interrupts for both encoders
  attachInterrupt(digitalPinToInterrupt(2), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(3), encoder2ISR, RISING);
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
    int delimiter5 = data.indexOf(',', delimiter4 + 1);
    if (delimiter1 != -1 && delimiter4 != -1) {
      int axisY = data.substring(0, delimiter1).toInt();
      int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
      int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
      int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
      int brake = data.substring(delimiter4 + 1, delimiter5).toInt();
      int buttons = data.substring(delimiter5 + 1).toInt();
            if (timeInterval()) {
        sendData();
      }
    if (axisY <= -25) {
      motorSpeed = map(axisY, -25, -512, 0, 50);
      forward();
      // Serial.println("Forward");
      } else if (axisY >= 25) {
        motorSpeed = map(axisY, 25, 512, 0, 50);
        reverse();
        // Serial.println("Reverse");
      }
    if (axisX <= -25) {
     motorSpeed = map(axisX, -25, -512, 0,50);
     left();
    //  Serial.println("Left");
      } else if (axisX >= 25) {
        motorSpeed = map(axisX, 25, 512, 0, 50);
        right();
        // Serial.println("Right");
      }
    if (throttle > 0) {
       motorSpeed = map(throttle, 0,1024, 0, 50);
       clockwise();
      //  Serial.println("Clockwise");
      }
    if (brake > 0) {
       motorSpeed = map(brake, 0,1024, 0, 50);
       anticlockwise();
      //  Serial.println("Antivclck");
      }
    if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
       stop();
        // Serial.println("STOP");
      }
if (buttons == 2 && !moveForwardFlag && !moveRightFlag) {
        totalDistance1 = 0;  // Reset distances
        totalDistance2 = 0;
        moveForwardFlag = true; // Start moving forward
        Serial.println("Starting Forward Motion...");
      }
    }
  }

  // Automatic Movement Sequence
  if (moveForwardFlag) {
    if (totalDistance2 <= 100) {
      moveForward(69);
    } else {
      stop();
      Serial.println("Forward Target Reached: 100 cm");
      moveForwardFlag = false;
      moveRightFlag = true; // Start moving right
    }
  } else if (moveRightFlag) {
    if (totalDistance1 <= 50) {
      moveRight(69);
    } else {
      stop();
      Serial.println("Right Target Reached: 50 cm");
      moveRightFlag = false; // Stop movement
    }
  }
}
void forward(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void reverse(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void left(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void right(){
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH);analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void clockwise(){
  digitalWrite(DIR1, HIGH);analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
}
void anticlockwise(){
  digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
}
void stop(){
  analogWrite(PWM1, 0);analogWrite(PWM2, 0);
  analogWrite(PWM3, 0); analogWrite(PWM4, 0);
}
void moveForward(int speed) {
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, speed);
  digitalWrite(DIR2, HIGH); analogWrite(PWM2, speed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, speed);
  digitalWrite(DIR4, HIGH); analogWrite(PWM4, speed);
  Serial.println("Moving Forward");
}
void moveRight(int speed) {
  digitalWrite(DIR1, HIGH); analogWrite(PWM1, speed);
  digitalWrite(DIR2, LOW); analogWrite(PWM2, speed);
  digitalWrite(DIR3, HIGH); analogWrite(PWM3, speed);
  digitalWrite(DIR4, LOW); analogWrite(PWM4, speed);
  Serial.println("Moving Right");
}
// Encoder ISR functions
void encoder1ISR() {
  counter1++;
}

void encoder2ISR() {
  counter2++;
}
void sendData() {
  float rpm1 = (float(counter1) * 60) / pulsesPerRevolution;
  float rpm2 = (float(counter2) * 60) / pulsesPerRevolution;

  distanceThisInterval1 = (float(counter1) / pulsesPerRevolution) * wheelCircumference;
  distanceThisInterval2 = (float(counter2) / pulsesPerRevolution) * wheelCircumference;

  totalDistance1 += distanceThisInterval1;
  totalDistance2 += distanceThisInterval2;

  Serial.print("Encoder 1 - RPM: ");
  Serial.print(rpm1);
  Serial.print("\t Distance this interval: ");
  Serial.print(distanceThisInterval1);
  Serial.print(" cm\t Total distance: ");
  Serial.print(totalDistance1);
  Serial.println(" cm");
  Serial.print("Encoder 2 - RPM: ");
  Serial.print(rpm2);
  Serial.print("\t Distance this interval: ");
  Serial.print(distanceThisInterval2);
  Serial.print(" cm\t Total distance: ");
  Serial.print(totalDistance2);
  Serial.println(" cm");

  resetSampling();
}

// Reset encoder counts
void resetSampling() {
  counter1 = 0;
  counter2 = 0;
}

// Timer function
boolean timeInterval() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    return true;
  }
  return false;
}