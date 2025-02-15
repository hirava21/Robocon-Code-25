#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Create an instance of the BNO085 sensor
BNO080 myIMU;

// Variable to store the initial yaw value
float initialYaw = 0;

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  // Debug: Print start message
  Serial.println("Starting setup...");

  // Initialize I2C communication
  Wire.begin();

  // Initialize the BNO085 sensor
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO085 not detected. Check your wiring or I2C address.");
    while (1); // Halt if sensor is not detected
  }

  Serial.println("BNO085 detected!");

  // Enable the rotation vector (quaternion) at 100Hz
  myIMU.enableRotationVector(100);

  // Wait for the sensor to stabilize
  delay(1000); // Optional: wait for a second to ensure sensor is ready

  // Read multiple initial yaw values to get a stable initial position
  float sumYaw = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable() == true) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();

      // Convert quaternion to Euler angles (initial yaw)
      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      // Convert radians to degrees
      yaw = yaw * 180.0 / PI;

      sumYaw += yaw;
      validReadings++;
    }
    delay(10); // Small delay between readings
  }

  // Calculate the average initial yaw
  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    Serial.print("Initial Yaw: ");
    Serial.println(initialYaw, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }

  // Debug: Print setup completion message
  Serial.println("Setup complete, starting loop...");
}

void loop() {
  // Debug: Print a message to see if the loop is running
  // Serial.println("Loop running...");

  // Check if new data is available
  if (myIMU.dataAvailable() == true) {
    // Read the quaternion data
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    // Convert quaternion to Euler angles (yaw, pitch, roll)
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // Convert radians to degrees
    yaw = yaw * 180.0 / PI;

    // Adjust the yaw by subtracting the initial yaw
    float adjustedYaw = yaw - initialYaw;

    // Ensure yaw stays within the range of -180 to 180 degrees
    if (adjustedYaw > 180) {
      adjustedYaw -= 360;
    } else if (adjustedYaw < -180) {
      adjustedYaw += 360;
    }

    // Print the adjusted yaw
    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);
  }

  // Add a small delay to avoid overwhelming the Serial monitor
  delay(500); // Slower delay for readability
}