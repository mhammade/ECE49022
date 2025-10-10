// Blink the built-in LED on pin 13

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // set built-in LED pin as output
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // turn LED on
  delay(1000);                      // wait 0.5 seconds
  digitalWrite(LED_BUILTIN, LOW);  // turn LED off
  delay(1000);                      // wait 0.5 seconds
}
