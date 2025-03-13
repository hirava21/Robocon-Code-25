#include <math.h>

// Pin assignments for motors
int PWM1 = 8; int DIR1 = 9;
int PWM2 = 10; int DIR2 = 11;
int PWM3 = 12; int DIR3 = 13;
int PWM4 = 6; int DIR4 = 7;

int centerX = 0; int centerY = 0;  // Joystick calibration centers
const int maxpwm = 110;  // Maximum motor speed
const int rotpwm = 50;   // Rotation speed scaling
const int deadZone = 25; // Dead zone for joystick input

void setup() {
  Serial.begin(9600);  // Monitor output
  Serial1.begin(9600); // UART communication with ESP32
  
  // Define motor control pins as outputs
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);
}

void loop() {
  if (Serial1.available() > 0) {
    // Read joystick, D-pad, throttle, and brake data
    String data = Serial1.readStringUntil('\n'); 
    int delimiter1 = data.indexOf(',');
    int delimiter2 = data.indexOf(',', delimiter1 + 1);
    int delimiter3 = data.indexOf(',', delimiter2 + 1);
    int delimiter4 = data.indexOf(',', delimiter3 + 1);

    // Parse the incoming data if it's valid
    if (delimiter1 != -1 && delimiter4 != -1) {
      int axisY = data.substring(0, delimiter1).toInt();
      int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
      int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();  // For rotation
      int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
      int brake = data.substring(delimiter4 + 1).toInt();

      // Adjust joystick values to remove offset
      int adjustedY = axisY - centerY;
      int adjustedX = axisX - centerX;

      // Apply a dead zone to avoid jittery movement
      if (abs(adjustedY) < deadZone && abs(adjustedX) < deadZone && abs(dpad) < deadZone) {
        // Stop all motors if within dead zone
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);
        analogWrite(PWM3, 0);
        analogWrite(PWM4, 0);
      } else {
        // Calculate the magnitudes for movement
        float angle = atan2(adjustedY, adjustedX) * 180 / PI; // Get angle of joystick direction
        int mag = constrain(sqrt(pow(adjustedX, 2) + pow(adjustedY, 2)), 0, maxpwm); // Magnitude of joystick input

        // Rotation control based on dpad or other input
        int rotation = constrain(dpad, -rotpwm, rotpwm);

        // Omni-wheel motor speed calculation
        // Motor placement is assumed to be at 45-degree angles or in an X-shape
        int speed1 = mag * cos((angle - 45) * PI / 180) + rotation;   // Front-right wheel
        int speed2 = mag * cos((angle + 45) * PI / 180) - rotation;   // Front-left wheel
        int speed3 = mag * cos((angle - 135) * PI / 180) + rotation;  // Rear-right wheel
        int speed4 = mag * cos((angle + 135) * PI / 180) - rotation;  // Rear-left wheel

        // Constrain motor speeds to allowable range (-maxpwm to maxpwm)
        speed1 = constrain(speed1, -maxpwm, maxpwm);
        speed2 = constrain(speed2, -maxpwm, maxpwm);
        speed3 = constrain(speed3, -maxpwm, maxpwm);
        speed4 = constrain(speed4, -maxpwm, maxpwm);

        // Set motor directions and speeds
        setMotorSpeed(PWM1, DIR1, speed1);
        setMotorSpeed(PWM2, DIR2, speed2);
        setMotorSpeed(PWM3, DIR3, speed3);
        setMotorSpeed(PWM4, DIR4, speed4);

        // Debugging: Print motor values
        String dataToSend = String(speed1) + "," + String(speed2) + "," + String(speed3) + "," + String(speed4);
        Serial.println("Motor PWM: " + dataToSend);
      }
    }
  }
  delay(50);  // Add a small delay to reduce data processing frequency
}

// Function to set motor speed and direction
void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);  // Forward direction
  } else {
    digitalWrite(dirPin, LOW);   // Reverse direction
    speed = -speed;              // Use absolute value for PWM
  }
  analogWrite(pwmPin, speed);    // Write PWM value to motor
}