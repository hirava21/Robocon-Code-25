#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

// Thresholds for filtering small movements and for correction decision (in radians)
const float rotationThreshold = 0.05;  
const float angleTolerance = 0.05;     

float angle = 0.0;  
unsigned long previousTime = 0;

void initializeSensor() {
  Wire1.begin();
  
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);  
  }
  Serial.println("BNO08x Found!");


  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);  
  }
}

void processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();


    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;


    if (abs(zRotation) > rotationThreshold) {
      angle += zRotation * deltaTime;  // theta = omega * deltaTime
    }


    Serial.print("Angle displaced: ");
    Serial.print(angle);
    Serial.print(" rad, ");


    if (abs(angle) > angleTolerance) {

      if (angle > 0) {
        Serial.println("Bot should go Clockwise to correct angle");
      } else {
        Serial.println("Bot should go Anticlockwise to correct angle");
      }
    } else {
      Serial.println("Angle is near zero, no correction needed.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  initializeSensor();
  previousTime = millis();  
}

void loop() {
  processGyroscopeData();
  delay(100);
}