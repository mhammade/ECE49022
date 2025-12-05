// === 3-Phase PWM: 62.5 kHz carrier, felec Hz electrical sine (UNO/Nano ATmega328P) ===
// Pins: Phase A -> 3 (OC2B), Phase B -> 9 (OC1A), Phase C -> 10 (OC1B)

#define phaseA 3
#define phaseB 9
#define phaseC 10

#define PI 3.14159265f

// ----- Speed control (electrical frequency in Hz) -----
float felec = 5.0f;      // electrical frequency in Hz (sine frequency for all phases)

// ----- Torque control (amplitude / duty scaling) -----
// 0.0  -> 0% duty (no torque)
// 1.0  -> full sine 0..100% duty
float dutyScale = 1.0f;  // change this (0..1) to control max duty / torque

// =========================
const uint16_t N = 256;       // number of samples per electrical period
uint8_t sineLUT[N];           // lookup table for sine duty cycles (0..255)

unsigned long stepIntervalUs; // time between two LUT steps (in microseconds)
unsigned long lastStepMicros = 0;
uint16_t idx = 0;             // index into sineLUT

// phase shifts: 120° and 240°
const uint16_t PHASE_120 = N / 3;     // 120° shift
const uint16_t PHASE_240 = 2 * N / 3; // 240° shift

// --- simple micros() implementation using Timer0 (64 prescaler, normal mode) ---
static volatile uint32_t microseconds = 0;

ISR(TIMER0_OVF_vect) {
  microseconds += 1024; // 256 ticks * 4 µs per tick @16 MHz, prescaler 64
}

void init_micros(void) {
  TCCR0A = 0x00;                   // normal mode
  TCCR0B = (1 << CS01) | (1 << CS00); // prescaler = 64
  TIMSK0 |= (1 << TOIE0);          // enable overflow interrupt
  sei();
}

unsigned long micros(void) {
  uint32_t m;
  uint8_t t;
  uint8_t oldSREG = SREG;

  cli();
  m = microseconds;
  t = TCNT0;
  if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
    m += 1024;
  }
  SREG = oldSREG;

  return m + (t * 4); // each tick ~4 µs
}

// helper to recompute step interval when felec changes
void updateStepInterval() {
  stepIntervalUs = (unsigned long)(1000000.0f / (felec * N));
}

void setup() {
  // configure pins as outputs:
  // Phase A -> PD3 (Arduino D3)
  // Phase B -> PB1 (Arduino D9)
  // Phase C -> PB2 (Arduino D10)
  DDRD |= (1 << DDD3);              // pinMode(phaseA, OUTPUT);
  DDRB |= (1 << DDB1) | (1 << DDB2); // pinMode(phaseB, OUTPUT); pinMode(phaseC, OUTPUT);

  init_micros();

  // =========================
  // Timer2 setup for OC2B (pin 3) -> Phase A
  // =========================
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= 0b00100000;       // COM2B1:0 = 10 -> non-inverting PWM on OC2B
  TCCR2A |= 0b00000011;       // WGM21:0 = 11 -> Fast PWM (TOP = 255)
  TCCR2B |= 0b00000001;       // CS22:0 = 001 -> prescaler = 1 (fPWM = 62.5 kHz)
  OCR2B = 0;

  // =========================
  // Timer1 setup for OC1A (pin 9) and OC1B (pin 10) -> Phases B and C
  // =========================
  TCCR1A = 0;
  TCCR1B = 0;

  // Non-inverting PWM on OC1A and OC1B
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

  // Fast PWM 8-bit (Mode 5: WGM13:0 = 0b0101, so WGM10=1, WGM12=1)
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);

  // Prescaler = 1 -> fPWM = 16 MHz / (1 * 256) = 62.5 kHz
  TCCR1B |= (1 << CS10);

  OCR1A = 0;   // Phase B
  OCR1B = 0;   // Phase C

  // =========================
  // Build sine LUT for one electrical period (0..255)
  // =========================
  for (uint16_t i = 0; i < N; i++) {
    float angle = 2.0f * PI * i / N;        // 0..2π
    float s = 0.5f * (sinf(angle) + 1.0f);  // map from [-1,1] to [0,1]
    sineLUT[i] = (uint8_t)(s * 255.0f);     // map to [0,255] for OCRx
  }

  // compute initial update interval for felec
  updateStepInterval();
}

void loop() {
  unsigned long now = micros();

  // (optional) Example: slowly ramp dutyScale between 0.3 and 1.0
  // to show torque change on the scope or motor.
  /*
  static unsigned long lastDutyUpdate = 0;
  static float dir = 1.0f;  // +1 or -1
  if (millis() - lastDutyUpdate > 50) {  // update every 50 ms
    lastDutyUpdate = millis();
    dutyScale += dir * 0.02f;
    if (dutyScale > 1.0f) { dutyScale = 1.0f; dir = -1.0f; }
    if (dutyScale < 0.3f) { dutyScale = 0.3f; dir = 1.0f; }
  }
  */

  // Update OCR registers at fixed intervals to create felec Hz sines
  if (now - lastStepMicros >= stepIntervalUs) {
    lastStepMicros = now;

    // base index for phase A
    uint16_t iA = idx;
    uint16_t iB = idx + PHASE_120;
    uint16_t iC = idx + PHASE_240;

    if (iB >= N) iB -= N;
    if (iC >= N) iC -= N;

    // ----- apply dutyScale (0..1) to each phase -----
    uint8_t aRaw = sineLUT[iA];
    uint8_t bRaw = sineLUT[iB];
    uint8_t cRaw = sineLUT[iC];

    OCR2B = (uint8_t)(aRaw * dutyScale);  // Phase A
    OCR1A = (uint8_t)(bRaw * dutyScale);  // Phase B
    OCR1B = (uint8_t)(cRaw * dutyScale);  // Phase C

    // advance base index
    idx++;
    if (idx >= N) {
      idx = 0;            // wrap around after one sine period
    }
  }

  // If you ever change felec at runtime, call:
  //   felec = newValue;
  //   updateStepInterval();
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
}
