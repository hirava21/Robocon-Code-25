#include <Servo.h>

Servo myBLDC;

void setup() {
  myBLDC.attach(9); // Attach ESC to pin 9
}

void loop() {
  // Gradually increase speed from 0 to 180 over 10 seconds
  for (int speed = 0; speed <= 90; speed++) {
    DriveBLDC(speed);  // Increase motor speed gradually
    delay(500);         // Wait for 50 milliseconds (adjust for ramp speed)
  }

  delay(2500); // Run the motor at full speed for 5 seconds

  // Gradually decrease speed back to 0 over 10 seconds
  // for (int speed = 180; speed >= 0; speed--) {
  //   DriveBLDC(speed);  // Decrease motor speed gradually
  //   delay(50);         // Wait for 50 milliseconds (adjust for ramp speed)
  

  StopBLDC(); // Stop the motor
  delay(5000); // Wait for 0.5 seconds before restarting
}

void StopBLDC() {
  myBLDC.write(0); // Send PWM signal to stop the motor
}

void DriveBLDC(int val1) {
  myBLDC.write(val1); // Send PWM signal to control motor speed
}
