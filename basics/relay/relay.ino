const int relayPin = 22;  

void setup() {
  pinMode(relayPin, OUTPUT);
}

void loop() {
  digitalWrite(relayPin, LOW);   
  delay(5000);   
  
  digitalWrite(relayPin, HIGH);
  delay(2000);                  
}
