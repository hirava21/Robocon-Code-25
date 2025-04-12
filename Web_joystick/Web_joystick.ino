#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "hirava21";  
const char* password = "hirava21";  

// Define motor control pins (use safe ESP32 pins)
const int DIR1 = 16, DIR2 = 17, DIR3 = 18, DIR4 = 19; 
const int PWM1 = 21, PWM2 = 22, PWM3 = 23, PWM4 = 25; 
const int motorSpeed = 100; 

WebServer server(80);  

const char html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Bot Joystick Control</title>
  <style>
    #joystick {
      width: 150px;
      height: 150px;
      background: lightgrey;
      border-radius: 50%;
      position: relative;
      margin: 50px auto;
      touch-action: none;
    }
    #stick {
      width: 50px;
      height: 50px;
      background: grey;
      border-radius: 50%;
      position: absolute;
      left: 50px;
      top: 50px;
      transition: 0.1s;
    }
  </style>
</head>
<body>
  <h1>ESP32 Bot Joystick Control</h1>
  <div id="joystick">
    <div id="stick"></div>
  </div>

  <script>
    const joystick = document.getElementById('joystick');
    const stick = document.getElementById('stick');

    joystick.addEventListener('touchmove', handleMove);
    joystick.addEventListener('mousemove', handleMove);
    joystick.addEventListener('touchend', stopBot);
    joystick.addEventListener('mouseup', stopBot);

    function handleMove(event) {
      let rect = joystick.getBoundingClientRect();
      let x = (event.touches ? event.touches[0].clientX : event.clientX) - rect.left - 75;
      let y = (event.touches ? event.touches[0].clientY : event.clientY) - rect.top - 75;
      
      let angle = Math.atan2(y, x);
      let distance = Math.min(Math.sqrt(x*x + y*y), 75);
      
      let moveX = distance * Math.cos(angle);
      let moveY = distance * Math.sin(angle);

      stick.style.left = `${moveX + 50}px`;
      stick.style.top = `${moveY + 50}px`;

      let command = '';

      if (moveY < -40 && Math.abs(moveX) < 30) {
        command = 'FORWARD';
      } else if (moveY > 40 && Math.abs(moveX) < 30) {
        command = 'REVERSE';
      } else if (moveX < -40 && Math.abs(moveY) < 30) {
        command = 'LEFT';
      } else if (moveX > 40 && Math.abs(moveY) < 30) {
        command = 'RIGHT';
      } else if (moveX > 30 && moveY < -30) {
        command = 'CW';  // Clockwise movement (diagonal top-right)
      } else if (moveX < -30 && moveY > 30) {
        command = 'CW';  // Clockwise movement (diagonal bottom-left)
      } else if (moveX < -30 && moveY < -30) {
        command = 'ACW'; // Anti-clockwise movement (diagonal top-left)
      } else if (moveX > 30 && moveY > 30) {
        command = 'ACW'; // Anti-clockwise movement (diagonal bottom-right)
      }

      if (command) sendCommand(command);
    }

    function stopBot() {
      stick.style.left = "50px";
      stick.style.top = "50px";
      sendCommand("STOP");
    }

    function sendCommand(cmd) {
      fetch('/' + cmd).then(response => console.log('Command Sent:', cmd));
    }
  </script>
</body>
</html>
)rawliteral";
// Handle the root page request
void handleRoot() {
  server.send(200, "text/html", html_page);
}
// Handle motor control commands
void handleCommand() {
  String command = server.uri().substring(1);

  if (command == "FORWARD") {
    digitalWrite(DIR1, HIGH); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, HIGH); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, HIGH); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, HIGH); ledcWrite(3, motorSpeed);
    Serial.println("Moving Forward");
  } 
  else if (command == "REVERSE") {
    digitalWrite(DIR1, LOW); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, LOW); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, LOW); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, LOW); ledcWrite(3, motorSpeed);
    Serial.println("Moving Reverse");
  } else if (command == "LEFT"){
    digitalWrite(DIR1, LOW); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, HIGH); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, LOW); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, HIGH); ledcWrite(3, motorSpeed);    
        Serial.println("L");
  } else if (command == "RIGHT"){
    digitalWrite(DIR1, HIGH); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, LOW); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, HIGH); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, LOW); ledcWrite(3, motorSpeed);
    Serial.println("R");
  } else if (command == "CW"){
    digitalWrite(DIR1, HIGH); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, LOW); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, LOW); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, HIGH); ledcWrite(3, motorSpeed);
    Serial.println("CW");
  } else if(command == "ACW"){
    digitalWrite(DIR1, LOW); ledcWrite(0, motorSpeed);
    digitalWrite(DIR2, HIGH); ledcWrite(1, motorSpeed);
    digitalWrite(DIR3, HIGH); ledcWrite(2, motorSpeed);
    digitalWrite(DIR4, LOW); ledcWrite(3, motorSpeed);
    Serial.println("ACW");
  }
  else if (command == "STOP") {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    Serial.println("Stopped");
  }

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(DIR1, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT); pinMode(DIR4, OUTPUT);

  // Set up PWM channels
  ledcSetup(0, 5000, 8);  // PWM channel, frequency, resolution
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);

  ledcAttachPin(PWM1, 0);
  ledcAttachPin(PWM2, 1);
  ledcAttachPin(PWM3, 2);
  ledcAttachPin(PWM4, 3);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi!");
  Serial.println(WiFi.localIP());

  // Define server routes
  server.on("/", handleRoot);
  server.on("/FORWARD", handleCommand);
  server.on("/REVERSE", handleCommand);
  server.on("/STOP", handleCommand);
  server.on("/RIGHT",handleCommand);
  server.on("/LEFT",handleCommand);
  server.on("/CW",handleCommand);
  server.on("/ACW",handleCommand);

  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();
}
