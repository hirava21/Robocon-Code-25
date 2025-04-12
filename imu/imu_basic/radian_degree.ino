#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Create an instance of the BNO085 sensor
BNO080 myIMU;
float initialYawRad = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("Starting setup...");
  Wire.begin();
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO085 not detected. Check your wiring or I2C address.");
    while (1); 
  }

  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);
  delay(1000); // Wait for the sensor to stabilize

  // Read multiple initial yaw values to get a stable initial position
  float sumYawRad = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable() == true) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();

      // Convert quaternion to yaw (in radians)
      float yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      sumYawRad += yawRad;
      validReadings++;
    }
    delay(10);
  }

  // Calculate the average initial yaw in radians
  if (validReadings > 0) {
    initialYawRad = sumYawRad / validReadings;
    Serial.print("Initial Yaw (radians): ");
    Serial.println(initialYawRad, 6);
    Serial.print("Initial Yaw (degrees): ");
    Serial.println(initialYawRad * 180.0 / PI, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }

  Serial.println("Setup complete, starting loop...");
}

// Function to read yaw in radians
void readYawRadians() {
  if (myIMU.dataAvailable() == true) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    float adjustedYawRad = yawRad - initialYawRad;

    if (adjustedYawRad < 0) {
      adjustedYawRad += 2 * PI;
    }

    Serial.print("radians: ");
    Serial.print(adjustedYawRad, 6);
    Serial.println(" ");
  }
}

// Function to read yaw in degrees
void readYawDegrees() {
  if (myIMU.dataAvailable() == true) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    float adjustedYawRad = yawRad - initialYawRad;

    if (adjustedYawRad < 0) {
      adjustedYawRad += 2 * PI;
    }

    float adjustedYawDeg = adjustedYawRad * 180.0 / PI;

    Serial.print("degree: ");
    Serial.print(adjustedYawDeg, 2);
    Serial.println("°");
  }
}

void loop() {
  readYawRadians();
  readYawDegrees();

  delay(500);
}
