#include <SPI.h>

// Pins
#define RX 0 
#define TX 1
#define MUX_A 4
#define MUX_B 8
#define MUX_C 2
#define Led 6
#define Fan 7
#define En_Phase 5
#define A_PHASE 9     // PWM-capable
#define B_PHASE 10    // PWM-capable
#define C_PHASE 3    // PWM-capable
// (Do not redefine MOSI/MISO/SCK — Arduino provides these)

// Telemetry
#define i_C A0
#define v_C A1
#define i_B A2
#define v_B A3
#define i_A A4
#define v_A A5

// LED
unsigned long BLINK_MS = 500;
unsigned long lastBlink = 0;
bool ledState = false;

// “telemetry” + UART
float Temp = 10;   // °C
int   Speed = 150;   // rpm
int   Torque = 50;  // Nm

bool driveEnabled = false;
const float T_OFF_HIGH = 100;   // turn off here or above
const float T_ON_HIGH  = 80;    // don't turn back on until at/below this
const float T_OFF_LOW  = -20;   // too cold: off
const float T_ON_LOW   = -10;   // warm enough to turn on again

// 0–255 sine table (60 samples). Replaced 256 -> 255
const uint8_t sineTable[60] = {
  128,141,154,167,180,192,203,214,224,233,
  241,248,253,255,255,255,253,248,241,233,
  224,214,203,192,180,167,154,141,128,114,
  101, 88, 75, 63, 52, 41, 31, 22, 14, 7,
  2, 0, 0, 0, 2, 7, 14, 22, 31, 41,
  52, 63, 75, 88,101,114,128,141,154,167
};

void idle_state(){
  digitalWrite(En_Phase, LOW);
  analogWrite(A_PHASE, 0);
  analogWrite(B_PHASE, 0);
  analogWrite(C_PHASE, 0);
}

void pwm3Phase(float speed, float torque) {
  static unsigned long lastMicros = 0;
  static int idx = 0;

  if (speed <= 0) return;  // avoid divide-by-zero

  // microseconds per sample (60 samples per electrical cycle)
  unsigned long stepDelay = 1000000UL / ( (unsigned long)(speed * 60) );

  if (micros() - lastMicros >= stepDelay) {
    lastMicros = micros();
    // map torque to 0..255 if you intend torque as a scalar amplitude
    int amp = constrain((int)torque, 0, 255);

    int a = (sineTable[idx] * amp) / 255;
    int b = (sineTable[(idx + 20) % 60] * amp) / 255; // 120°
    int c = (sineTable[(idx + 40) % 60] * amp) / 255; // 240°
    analogWrite(A_PHASE, a);
    analogWrite(B_PHASE, b);
    analogWrite(C_PHASE, c);
    idx = (idx + 1) % 60;
  }
}

void working_state(){
  digitalWrite(En_Phase, HIGH);
  // (Serial parsing stub kept commented)
  pwm3Phase(Speed, Torque);
}

void setup() {
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);

  pinMode(Led, OUTPUT);
  digitalWrite(Led, HIGH);

  pinMode(Fan, OUTPUT);

  pinMode(En_Phase, OUTPUT);
  pinMode(A_PHASE, OUTPUT);
  pinMode(B_PHASE, OUTPUT);
  pinMode(C_PHASE, OUTPUT);

  idle_state();
}

void loop() {
  // Fan control
  if (Temp > 60)       digitalWrite(Fan, HIGH);
  else if (Temp < 30)  digitalWrite(Fan, LOW);

// Hysteresis-based thermal control
//Use Telemetry



//
// Update state
if (Temp >= T_OFF_HIGH || Temp <= T_OFF_LOW) {
  driveEnabled = false;                 // hard OFF at extremes
} else if (!driveEnabled && (Temp <= T_ON_HIGH && Temp >= T_ON_LOW)) {
  driveEnabled = true;                  // only re-enable in the safe band
}

// Act on state
if (driveEnabled) {
  BLINK_MS = 1000;
  working_state();
} else {
  BLINK_MS = 200;
  idle_state();
}

  // LED blink
  unsigned long now = millis();
  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(Led, ledState ? HIGH : LOW);
  }
}
