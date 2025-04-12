#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// Create the BNO08x object
Adafruit_BNO08x bno;

// Variable to hold sensor events
sh2_SensorValue_t sensorValue;

// Threshold to filter out small movements (in radians/second)
const float rotationThreshold = 0.05;

// Variables to store the angle displacement
float angle = 0.0;  // Angle in radians
unsigned long previousTime = 0;  // Store the last time for delta time calculation

// Initialize the sensor
void initializeSensor() {
  // Initialize I2C communication on Arduino Mega 2560
  Wire.begin();  // Use the default SDA (pin 20) and SCL (pin 21)

  // Attempt to initialize the sensor
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);  // Stop if sensor initialization fails
  }

  Serial.println("BNO08x Found!");

  // Enable gyroscope data report
  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);  // Stop if enabling the gyroscope fails
  }
}

// Function to handle gyroscope data and detect rotation
void processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;

    // Get the current time
    unsigned long currentTime = millis();
    
    // Calculate delta time (in seconds)
    float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert ms to seconds

    // Update the angle (integrate the gyroscope data)
    angle += zRotation * deltaTime;  // theta = omega * deltaTime

    // Update previous time for next iteration
    previousTime = currentTime;

    // Output the angle and rotation direction
    Serial.print("Angle displaced: ");
    Serial.print(angle);
    Serial.print(" rad, ");

    // Determine the direction based on the Z-axis value
    if (abs(zRotation) > rotationThreshold) {
      if (zRotation > 0) {
        // Anticlockwise direction (positive Z)
        Serial.println("Bot should go in Clockwise direction");
      } else {
        // Clockwise direction (negative Z)
        Serial.println("Bot should go in Anticlockwise direction");
      }
    } else {
      Serial.println("No significant rotation detected.");
    }
  }
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the BNO08x sensor
  initializeSensor();
}

void loop() {
  // Continuously process gyroscope data
  processGyroscopeData();

  delay(100);  // Small delay between readings to avoid overloading the serial output
}
