#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "hirava21";  // Replace with your WiFi SSID
const char* password = "hirava21";  // Replace with your WiFi password

const int DIR1 = 9, DIR2 = 11, DIR3 = 13, DIR4 = 7; 
const int PWM1 = 8, PWM2 = 10, PWM3 = 12, PWM4 = 6; 
const int motorSpeed = 100; // PWM value (0-255)

WebServer server(80);  // Start server on port 80

// HTML page content
const char html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Bot Control</title>
  <style>
    button { padding: 15px; font-size: 20px; margin: 10px; }
  </style>
</head>
<body>
  <h1>ESP32 Bot Control</h1>
  <button onclick="sendCommand('FORWARD')">Forward</button>
  <button onclick="sendCommand('REVERSE')">Reverse</button>
  <button onclick="sendCommand('LEFT')">Left</button>
  <button onclick="sendCommand('RIGHT')">Right</button>
  <button onclick="sendCommand('CW')">Clockwise</button>
  <button onclick="sendCommand('ACW')">Anti-Clockwise</button>
  <button onclick="sendCommand('STOP')">Stop</button>

  <script>
    function sendCommand(cmd) {
      fetch('/' + cmd).then(response => console.log('Command Sent:', cmd));
    }
  </script>
</body>
</html>
)rawliteral";

// Handle incoming requests
void handleRoot() {
  server.send(200, "text/html", html_page);
}

// Function to execute motor commands
void handleCommand() {
  String command = server.uri().substring(1);

  if (command == "FORWARD") {
      digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
    Serial.println("F");
  } else if (command == "REVERSE") {
      Serial.println("B");
      digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
  } else if (command == "LEFT") {
      Serial.println("L");
      digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed); 
  } else if (command == "RIGHT") {
      Serial.println("R");
      digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
  } else if (command == "CW") {
      Serial.println("CW");
      digitalWrite(DIR1, HIGH); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, LOW); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, LOW); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, HIGH); analogWrite(PWM4, motorSpeed);
  } else if (command == "ACW") {
      Serial.println("CCW");
      digitalWrite(DIR1, LOW); analogWrite(PWM1, motorSpeed);
      digitalWrite(DIR2, HIGH); analogWrite(PWM2, motorSpeed);
      digitalWrite(DIR3, HIGH); analogWrite(PWM3, motorSpeed);
      digitalWrite(DIR4, LOW); analogWrite(PWM4, motorSpeed);
  } else if (command == "STOP") {
analogWrite(PWM1, 0);
analogWrite(PWM2, 0);
analogWrite(PWM3, 0);
analogWrite(PWM4, 0);
  }

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
    // Configure motor pins
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi!");
  Serial.println(WiFi.localIP());  // Print ESP32 IP address

  // Define server routes
  server.on("/", handleRoot);
  server.onNotFound(handleCommand);

  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();
}
