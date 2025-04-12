#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;
float initialYaw = 0;
float previousYaw = 0; // Store the last yaw value
unsigned long previousTime = 0; // Store the last timestamp

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
  delay(1000);

  // Calculate initial yaw
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

  previousTime = millis(); // Initialize time tracking
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

    if (adjustedYaw > 180) adjustedYaw -= 360;
    else if (adjustedYaw < -180) adjustedYaw += 360;

    // Get current time
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    if (deltaTime > 0) { // Avoid division by zero
      float deltaYaw = adjustedYaw - previousYaw;

      // Handle wrap-around at -180/180 degrees
      if (deltaYaw > 180) deltaYaw -= 360;
      else if (deltaYaw < -180) deltaYaw += 360;

      float yawRate = deltaYaw / deltaTime; // Degrees per second

      Serial.print("Adjusted Yaw: ");
      Serial.print(adjustedYaw, 2);
      Serial.print(" | Yaw Rate: ");
      Serial.print(yawRate, 2);
      Serial.println(" deg/s");

      // Update previous values
      previousYaw = adjustedYaw;
      previousTime = currentTime;
    }
  }

  delay(50); // Faster updates for yaw rate calculation
}
