
int frontRightMotor = 8;
int frontLeftMotor = 9;
int backRightMotor = 10;
int backLeftMotor = 11;

// Declare PWM pins for speed control
int frontRightSpeed = 3;  // PWM pin for Front Right motor speed
int frontLeftSpeed = 5;   // PWM pin for Front Left motor speed
int backRightSpeed = 6;   // PWM pin for Back Right motor speed
int backLeftSpeed = 11;   // PWM pin for Back Left motor speed

void setup() {
  
  pinMode(frontRightMotor, OUTPUT);
  pinMode(frontLeftMotor, OUTPUT);
  pinMode(backRightMotor, OUTPUT);
  pinMode(backLeftMotor, OUTPUT);

  pinMode(frontRightSpeed, OUTPUT);
  pinMode(frontLeftSpeed, OUTPUT);
  pinMode(backRightSpeed, OUTPUT);
  pinMode(backLeftSpeed, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Test motor movements with delays
  forward();
  delay(1000);

  right();
  delay(1000);

  left();
  delay(1000);

  backward();
  delay(1000);

  stop();
  delay(1000);
}

void forward() {
  Serial.println("Moving Forward");
  // Set motor direction
  digitalWrite(frontRightMotor, HIGH);  // Front Right Motor ON
  digitalWrite(frontLeftMotor, LOW);    // Front Left Motor OFF
  digitalWrite(backRightMotor, LOW);    // Back Right Motor OFF
  digitalWrite(backLeftMotor, HIGH);    // Back Left Motor ON

  // Set motor speeds using PWM (range: 0 to 255)
  analogWrite(frontRightSpeed, 255); // Full speed for Front Right motor
  analogWrite(frontLeftSpeed, 0);    // No speed for Front Left motor
  analogWrite(backRightSpeed, 0);    // No speed for Back Right motor
  analogWrite(backLeftSpeed, 255);   // Full speed for Back Left motor
}

void right() {
  Serial.println("Turning Right");
  // Set motor direction for turning right
  digitalWrite(frontRightMotor, HIGH);  // Front Right Motor ON
  digitalWrite(frontLeftMotor, LOW);    // Front Left Motor OFF
  digitalWrite(backRightMotor, HIGH);   // Back Right Motor ON
  digitalWrite(backLeftMotor, LOW);     // Back Left Motor OFF

  // Set motor speeds using PWM (range: 0 to 255)
  analogWrite(frontRightSpeed, 255);  // Full speed for Front Right motor
  analogWrite(frontLeftSpeed, 0);     // No speed for Front Left motor
  analogWrite(backRightSpeed, 255);   // Full speed for Back Right motor
  analogWrite(backLeftSpeed, 0);      // No speed for Back Left motor
}

void left() {
  Serial.println("Turning Left");
  // Set motor direction for turning left
  digitalWrite(frontRightMotor, LOW);   // Front Right Motor OFF
  digitalWrite(frontLeftMotor, HIGH);   // Front Left Motor ON
  digitalWrite(backRightMotor, LOW);    // Back Right Motor OFF
  digitalWrite(backLeftMotor, HIGH);    // Back Left Motor ON

  // Set motor speeds using PWM (range: 0 to 255)
  analogWrite(frontRightSpeed, 0);  // No speed for Front Right motor
  analogWrite(frontLeftSpeed, 255); // Full speed for Front Left motor
  analogWrite(backRightSpeed, 0);   // No speed for Back Right motor
  analogWrite(backLeftSpeed, 255);  // Full speed for Back Left motor
}

void backward() {
  Serial.println("Moving Backward");
  // Set motor direction for moving backward
  digitalWrite(frontRightMotor, LOW);   // Front Right Motor OFF
  digitalWrite(frontLeftMotor, HIGH);   // Front Left Motor ON
  digitalWrite(backRightMotor, HIGH);   // Back Right Motor ON
  digitalWrite(backLeftMotor, LOW);     // Back Left Motor OFF

  // Set motor speeds using PWM (range: 0 to 255)
  analogWrite(frontRightSpeed, 0);   // No speed for Front Right motor
  analogWrite(frontLeftSpeed, 255);  // Full speed for Front Left motor
  analogWrite(backRightSpeed, 255);  // Full speed for Back Right motor
  analogWrite(backLeftSpeed, 0);     // No speed for Back Left motor
}

void stop() {
  Serial.println("Stopping Motors");
  // Stop all motors
  digitalWrite(frontRightMotor, LOW);   // Front Right Motor OFF
  digitalWrite(frontLeftMotor, LOW);    // Front Left Motor OFF
  digitalWrite(backRightMotor, LOW);    // Back Right Motor OFF
  digitalWrite(backLeftMotor, LOW);     // Back Left Motor OFF

  // Set motor speeds to 0 (stop motors)
  analogWrite(frontRightSpeed, 0);  // Stop Front Right motor
  analogWrite(frontLeftSpeed, 0);   // Stop Front Left motor
  analogWrite(backRightSpeed, 0);   // Stop Back Right motor
  analogWrite(backLeftSpeed, 0);    // Stop Back Left motor
}