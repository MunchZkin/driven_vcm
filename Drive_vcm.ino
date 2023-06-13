const int ledPin =  LED_BUILTIN;

// Variables will change:
int ledState = LOW;

unsigned long previousMillis_1kHz = 0;
unsigned long previousMillis_200Hz = 0;
unsigned long previousMillis_50Hz = 0;
unsigned long previousMillis_1Hz = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1ms (1khz) task time
  if (currentMillis - previousMillis_1kHz >= 1) {
    previousMillis_1kHz = currentMillis;
    
    // enter code here
  }

  // 5ms (200Hz) task time
  if (currentMillis - previousMillis_200Hz >= 5) {
    previousMillis_200Hz = currentMillis;
    
    // enter code here
  }
    
  // 20ms (50Hz) task timee
  if (currentMillis - previousMillis_50Hz >= 20) {
    previousMillis_50Hz = currentMillis;
    
    // enter code here
  }

  // 1000ms (1Hz) task timee
  if (currentMillis - previousMillis_1Hz >= 1000) {
    previousMillis_1Hz = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  
}
