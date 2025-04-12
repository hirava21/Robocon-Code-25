#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Create an instance of the BNO085 sensor
BNO080 myIMU;

float initialYaw = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting setup...");

  // Initialize I2C communication on Wire1 (SDA2/SCL2 on Teensy 4.1)
  Wire1.begin(); // <-- CHANGED THIS LINE

  // Initialize the BNO085 sensor on Wire1
  if (myIMU.begin(0x4A, Wire1) == false) { // <-- CHANGED THIS LINE
    Serial.println("BNO085 not detected. Check your wiring or I2C address.");
    while (1);
  }

  Serial.println("BNO085 detected!");

  myIMU.enableRotationVector(100);
  delay(1000);

  float sumYaw = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();

      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;

      sumYaw += yaw;
      validReadings++;
    }
    delay(10);
  }

  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    Serial.print("Initial Yaw: ");
    Serial.println(initialYaw, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }

  Serial.println("Setup complete, starting loop...");
}

void loop() {
  if (myIMU.dataAvailable()) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;

    float adjustedYaw = yaw - initialYaw;

    if (adjustedYaw > 180) {
      adjustedYaw -= 360;
    } else if (adjustedYaw < -180) {
      adjustedYaw += 360;
    }

    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);
  }

  delay(500);
}
