volatile unsigned int counter = 0;  //This variable will increase on the rotation of encoder

unsigned long lastTime = 0;         // will store last time value was sendt
const long interval = 1000;         // interval at which to send result in milli second.  1000 ms = 1 second


void setup() {
  Serial.begin (9600);

  pinMode(2, INPUT);           // set pin to input
  digitalWrite(2, HIGH);       // turn on pullup resistors

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

}

void loop() {
  
  // Chedk if it's time to sendt data
  if(timeIntervall()) {
    sendData();
    resetSampling();
    
  }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH

  counter++;
}

void sendData() {
  Serial.print("Pulse pr second = ");
  Serial.print(counter);      // Sending cout / time

  float speedOut = map(counter, 0, 1200, 0, 100);      // change from 400 pulse pr / second to 100 m/h

  Serial.print("\t Speed = ");
  Serial.println(speedOut);

}

void resetSampling() {
  counter = 0;

}

boolean timeIntervall() {
  unsigned long currentTime = millis();

  // Chedk if it's time to make an interupt
  if (currentTime - lastTime >= interval) {
    lastTime = lastTime + interval;

    return true;
  } else if (currentTime < lastTime) {
    // After 50 day millis() will start 0 again
    lastTime = 0;

  } else {
    return false;
  }

}