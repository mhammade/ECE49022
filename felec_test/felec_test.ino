const int pA = 3;
const int pB = 9;
const int pC = 10;

const unsigned long T = 10000;     // 100 Hz → 10 000 µs
const unsigned long duty = T / 2;  // 50% → 5 000 µs
const unsigned long shiftB = T / 3;      // 3 333 µs
const unsigned long shiftC = 2 * T / 3;  // 6 666 µs

void setup() {
  pinMode(pA, OUTPUT);
  pinMode(pB, OUTPUT);
  pinMode(pC, OUTPUT);
}

void loop() {
  unsigned long t0 = micros();

  // 0 ms → phase A HIGH
  digitalWrite(pA, HIGH);

  // 3.33 ms → phase B HIGH
  while ((micros() - t0) < shiftB) {}
  digitalWrite(pB, HIGH);

  // 6.67 ms → phase C HIGH
  while ((micros() - t0) < shiftC) {}
  digitalWrite(pC, HIGH);

  // 5 ms → phase A LOW
  while ((micros() - t0) < duty) {}
  digitalWrite(pA, LOW);

  // 3.33 ms + 5 ms = 8.33 ms → phase B LOW
  while ((micros() - t0) < (shiftB + duty)) {}
  digitalWrite(pB, LOW);

  // 6.67 ms + 5 ms = 11.67 ms → phase C LOW
  // BUT our period is only 10 ms → so this LOW will happen in next cycle
  while ((micros() - t0) < (shiftC + duty)) {}
  digitalWrite(pC, LOW);

  // wait until full 10 ms is done
  while ((micros() - t0) < T) {}
}
