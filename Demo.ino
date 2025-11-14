void setup() {
  Serial.begin(9600);     // simple baud rate
  pinMode(6, OUTPUT);    // onboard LED
  Serial.println("Ready! Type 1=ON, 0=OFF");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();   // read one byte

    Serial.print("Got: ");
    Serial.println(c);        // echo the character

    if (c == '1') {
      digitalWrite(6, HIGH);
      Serial.println("LED ON");
    }

    if (c == '0') {
      digitalWrite(6, LOW);
      Serial.println("LED OFF");
    }
  }
}
