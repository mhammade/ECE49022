#define F_CPU 16000000UL

// === 3-Phase PWM + Rail / Current / Temp Monitor (UNO/Nano ATmega328P) ===
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>

#define phaseA 3
#define phaseB 9
#define phaseC 10
#define phaseEn 5
#define LED_PIN 6
#define FAN_PIN 7

// digital levels & modes
#define HIGH   1
#define LOW    0
#define OUTPUT 1
#define INPUT  0

// Arduino-style analog pin numbers
#define A0 14
#define A2 16
#define A6 20

#define PI 3.14159265f

// UART config
#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)

// Current sense pins
const int PC0_PIN = A0; // i-sns-C
const int PC2_PIN = A2; // i-sns-B

// MUX (DG4051) + ADC pins  (A = LSB, C = MSB)
const int MUX_A = 4;    // DG4051 A0 (LSB)
const int MUX_B = 8;    // DG4051 A1
const int MUX_C = 2;    // DG4051 A2 (MSB)
const int MUX_OUT_PIN = A6;

const float VREF = 5.0;
const int   NUM_SAMPLES = 8;

// PWM + sine LUT
float felec = 5.0f;
float dutyScale = 1.0f;
const uint16_t N = 256;
uint8_t  sineLUT[256];
uint16_t idx = 0;
unsigned long stepIntervalUs, lastStepMicros = 0;
const uint16_t PHASE_120 = 256 / 3;
const uint16_t PHASE_240 = 2 * 256 / 3;

bool motorOn = false;

// Serial state
typedef enum {
    WAITING_FOR_COMMAND,
    READING_SPEED,
    READING_TORQUE,
    READING_ONOFF
} InputState;

InputState state = WAITING_FOR_COMMAND;

#define INPUT_BUFFER_SIZE 32
char inputBuffer[INPUT_BUFFER_SIZE];
uint8_t inputLen = 0;

// Rail specs  (tolerance now ±0.20 V)
struct RailSpec {
    const char* name;
    uint8_t channel;
    float expectedVolts;
    float tolerance;
};

typedef struct RailSpec RailSpec;

RailSpec rails[] = {
  { "Sense_0", 3, 4.00f, 0.20f },   // V-sns-24V
  { "Sense_1", 0, 4.36f, 0.20f },   // V-sns-boost (non-fatal warning)
  { "Sense_2", 1, 4.00f, 0.20f },   // V-sns-12V
  { "Sense_3", 2, 2.50f, 0.20f },   // V-sns-2.5V (fatal)
  { "Sense_5", 5, 4.54f, 0.20f },   // V-sns-5V
};

const int NUM_RAILS = sizeof(rails) / sizeof(rails[0]);

// -------- Timer0 for millis()/micros() ----------
volatile uint32_t timer0_millis = 0;
volatile uint32_t timer0_micros = 0;

ISR(TIMER0_OVF_vect) {
    // Timer0: prescaler 64, 16 MHz -> 250 kHz (4 µs per tick)
    // 256 ticks/overflow => 1024 µs
    timer0_millis++;
    timer0_micros += 1024;
}

static void timer0_init(void) {
    // Fast PWM, prescaler 64 (like Arduino core)
    TCCR0A = (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << TOIE0);
}

unsigned long millis(void) {
    uint8_t sreg = SREG;
    unsigned long m;
    cli();
    m = timer0_millis;
    SREG = sreg;
    return m;
}

unsigned long micros(void) {
    uint8_t sreg = SREG;
    unsigned long m;
    uint8_t t;
    cli();
    m = timer0_micros;
    t = TCNT0;
    SREG = sreg;
    // each tick = 4 µs
    return m + ((unsigned long)t * 4UL);
}

// -------- UART (printf over UART0) ----------
static int uart_putchar(char c, FILE* stream) {
    if (c == '\n') {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = '\r';
    }
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8N1
}

static int uart_rx_available(void) {
    return (UCSR0A & (1 << RXC0));
}

static char uart_getchar_nowait(void) {
    return UDR0;
}

// -------- GPIO helpers (Arduino-like) ----------

void pinMode(uint8_t pin, uint8_t mode) {
    switch (pin) {
    case 2: // D2 -> PD2
        if (mode == OUTPUT) DDRD |= (1 << DDD2);
        else DDRD &= ~(1 << DDD2);
        break;
    case 3: // D3 -> PD3
        if (mode == OUTPUT) DDRD |= (1 << DDD3);
        else DDRD &= ~(1 << DDD3);
        break;
    case 4: // D4 -> PD4
        if (mode == OUTPUT) DDRD |= (1 << DDD4);
        else DDRD &= ~(1 << DDD4);
        break;
    case 5: // D5 -> PD5
        if (mode == OUTPUT) DDRD |= (1 << DDD5);
        else DDRD &= ~(1 << DDD5);
        break;
    case 6: // D6 -> PD6
        if (mode == OUTPUT) DDRD |= (1 << DDD6);
        else DDRD &= ~(1 << DDD6);
        break;
    case 7: // D7 -> PD7
        if (mode == OUTPUT) DDRD |= (1 << DDD7);
        else DDRD &= ~(1 << DDD7);
        break;
    case 8: // D8 -> PB0
        if (mode == OUTPUT) DDRB |= (1 << DDB0);
        else DDRB &= ~(1 << DDB0);
        break;
    case 9: // D9 -> PB1
        if (mode == OUTPUT) DDRB |= (1 << DDB1);
        else DDRB &= ~(1 << DDB1);
        break;
    case 10: // D10 -> PB2
        if (mode == OUTPUT) DDRB |= (1 << DDB2);
        else DDRB &= ~(1 << DDB2);
        break;

        // Analog pins used as GPIO (here just for completeness)
    case A0: // PC0
        if (mode == OUTPUT) DDRC |= (1 << DDC0);
        else DDRC &= ~(1 << DDC0);
        break;
    case A2: // PC2
        if (mode == OUTPUT) DDRC |= (1 << DDC2);
        else DDRC &= ~(1 << DDC2);
        break;

    default:
        break;
    }
}

void digitalWrite(uint8_t pin, uint8_t val) {
    switch (pin) {
    case 2:
        if (val == HIGH) PORTD |= (1 << PORTD2);
        else PORTD &= ~(1 << PORTD2);
        break;
    case 3:
        if (val == HIGH) PORTD |= (1 << PORTD3);
        else PORTD &= ~(1 << PORTD3);
        break;
    case 4:
        if (val == HIGH) PORTD |= (1 << PORTD4);
        else PORTD &= ~(1 << PORTD4);
        break;
    case 5:
        if (val == HIGH) PORTD |= (1 << PORTD5);
        else PORTD &= ~(1 << PORTD5);
        break;
    case 6:
        if (val == HIGH) PORTD |= (1 << PORTD6);
        else PORTD &= ~(1 << PORTD6);
        break;
    case 7:
        if (val == HIGH) PORTD |= (1 << PORTD7);
        else PORTD &= ~(1 << PORTD7);
        break;
    case 8:
        if (val == HIGH) PORTB |= (1 << PORTB0);
        else PORTB &= ~(1 << PORTB0);
        break;
    case 9:
        if (val == HIGH) PORTB |= (1 << PORTB1);
        else PORTB &= ~(1 << PORTB1);
        break;
    case 10:
        if (val == HIGH) PORTB |= (1 << PORTB2);
        else PORTB &= ~(1 << PORTB2);
        break;
    default:
        break;
    }
}

// -------- ADC (analogRead-style) ----------

void adc_init(void) {
    ADMUX = (1 << REFS0);                   // AVcc ref
    ADCSRA = (1 << ADEN) |                   // enable ADC
        (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

// Arduino-like analogRead: pin -> ADC channel
int analogRead(uint8_t pin) {
    uint8_t channel;

    if (pin >= 14) {         // A0..A7 -> 14..21
        channel = pin - 14;
    }
    else {
        channel = pin;
    }

    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));  // wait
    return ADC;
}

// ---------- helper logic from your sketch ----------

void updateStepInterval() {
    stepIntervalUs = (unsigned long)(100000000.0f / (felec * N));
}

void selectMuxChannel(uint8_t ch) {
    // bit0 -> A0(LSB)=MUX_A, bit1 -> A1=MUX_B, bit2 -> A2(MSB)=MUX_C
    digitalWrite(MUX_A, (ch & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_B, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_C, (ch & 0x04) ? HIGH : LOW);
    _delay_us(10);   // small settling time
}

int readADC(uint8_t pin) {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(pin);
    }
    return (int)(sum / NUM_SAMPLES);
}

float adcToVolts(int raw) {
    return raw * VREF / 1023.0f;
}

// MCP9700A on mux channel 6
int readMuxTemp() {
    selectMuxChannel(6);          // 6 = 0b110 -> C=1,B=1,A=0
    return analogRead(MUX_OUT_PIN);
}

float tempC_fromRaw(int raw) {
    float v = adcToVolts(raw);
    return (v - 0.5f) * 100.0f;
}
float tempF_fromRaw(int raw) {
    float c = tempC_fromRaw(raw);
    return c * 9.0f / 5.0f + 32.0f;
}

// -------- input buffer helpers ----------

static void resetInputBuffer(void) {
    inputLen = 0;
    inputBuffer[0] = '\0';
}

void setup(void) {
    // PWM pins
    pinMode(phaseB, OUTPUT);
    pinMode(phaseC, OUTPUT);
    pinMode(phaseEn, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, HIGH);
    digitalWrite(phaseEn, LOW);

    // current sense pins (ADC only; DDR defaults to input)
    // pinMode(PC0_PIN, INPUT);
    // pinMode(PC2_PIN, INPUT);

    // MUX pins
    pinMode(MUX_A, OUTPUT);
    pinMode(MUX_B, OUTPUT);
    pinMode(MUX_C, OUTPUT);

    // Timer2: OC2B (D3) 62.5 kHz
    TCCR2A = 0; TCCR2B = 0;
    TCCR2A |= 0b00100000;  // OC2B non-inverting
    TCCR2A |= 0b00000011;  // Fast PWM 0..255
    TCCR2B |= 0b00000001;  // prescaler 1
    OCR2B = 0;

    // Timer1: OC1A (D9), OC1B (D10) 62.5 kHz
    TCCR1A = 0; TCCR1B = 0;
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    TCCR1A |= (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    OCR1A = 0; OCR1B = 0;

    // Sine LUT
    for (uint16_t i = 0; i < N; i++) {
        float a = 2.0f * PI * i / N;
        float s = 0.5f * (sinf(a) + 1.0f);
        sineLUT[i] = (uint8_t)(s * 255.0f);
    }
    updateStepInterval();

    pinMode(LED_PIN, OUTPUT);

    printf("3-Phase PWM + Rail/Current/Temp Monitor\n");
    printf("Commands: s=Speed  t=Torque  ON/OFF=PWM enable\n");
}

void handleSerial(void) {
    while (uart_rx_available()) {
        char c = uart_getchar_nowait();
        if (c == '\r') continue;

        if (state == WAITING_FOR_COMMAND) {
            if (c == 's' || c == 'S') {
                state = READING_SPEED;
                resetInputBuffer();
                printf("Speed 5–250:\n");
            }
            else if (c == 't' || c == 'T') {
                state = READING_TORQUE;
                resetInputBuffer();
                printf("Torque 0.5–2.0:\n");
            }
            else if (c == 'O' || c == 'o') {
                state = READING_ONOFF;
                resetInputBuffer();
                printf("Type ON or OFF:\n");
            }
            else if (c == '\n') {
                // ignore
            }
            else {
                printf("Unknown '%c'\n", c);
            }
        }
        else {
            if (c == '\n') {
                if (inputLen == 0) {
                    printf("Empty input.\n");
                }
                else if (state == READING_SPEED) {
                    int v = atoi(inputBuffer);
                    if (v < 5 || v > 250) {
                        printf("\nSpeed out of range.\n");
                    }
                    else {
                        felec = (float)v;
                        updateStepInterval();
                        printf("\nSpeed=%.1f\n", felec);
                    }
                }
                else if (state == READING_TORQUE) {
                    float v = (float)atof(inputBuffer);
                    if (v < 0.5f || v > 2.0f) {
                        printf("\nTorque out of range.\n");
                    }
                    else {
                        dutyScale = v / 2.0f;
                        printf("\nTorque scale=%.3f\n", dutyScale * 2.0f);
                    }
                }
                else if (state == READING_ONOFF) {
                    for (uint8_t i = 0; i < inputLen; i++) {
                        inputBuffer[i] = (char)toupper((unsigned char)inputBuffer[i]);
                    }
                    if (strcmp(inputBuffer, "ON") == 0) {
                        motorOn = true;
                        digitalWrite(phaseEn, HIGH);
                        printf("\nPWM ON\n");
                    }
                    else if (strcmp(inputBuffer, "OFF") == 0) {
                        motorOn = false;
                        digitalWrite(phaseEn, LOW);
                        OCR2B = 0;
                        OCR1A = 0;
                        OCR1B = 0;
                        printf("\nPWM OFF\n");
                    }
                    else {
                        printf("\nUse ON or OFF.\n");
                    }
                }
                state = WAITING_FOR_COMMAND;
                resetInputBuffer();
            }
            else {
                if (inputLen < (INPUT_BUFFER_SIZE - 1)) {
                    inputBuffer[inputLen++] = c;
                    inputBuffer[inputLen] = '\0';
                }
                putchar(c);  // echo
            }
        }
    }
}

void loop(void) {
    unsigned long now = micros();

    // 3-phase sine PWM
    if (now - lastStepMicros >= stepIntervalUs) {
        lastStepMicros = now;
        uint16_t iA = idx;
        uint16_t iB = idx + PHASE_120; if (iB >= N) iB -= N;
        uint16_t iC = idx + PHASE_240; if (iC >= N) iC -= N;

        uint8_t aRaw = sineLUT[iA];
        uint8_t bRaw = sineLUT[iB];
        uint8_t cRaw = sineLUT[iC];

        if (motorOn) {
            OCR2B = (uint8_t)(aRaw * dutyScale);
            OCR1A = (uint8_t)(bRaw * dutyScale);
            OCR1B = (uint8_t)(cRaw * dutyScale);
        }
        else {
            OCR2B = 0;
            OCR1A = 0;
            OCR1B = 0;
        }

        idx++;
        if (idx >= N) idx = 0;
    }

    handleSerial();

    // --------- Every 10 seconds: temp + rails + currents ---------
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000000UL) {   // NOTE: 1,000,000 ms (~16.7 min) as in your code
        lastPrint = millis();

        printf("\n=== Status =================================\n");

        // Currents B/C from PC2/PC0
        int rawC = analogRead(PC0_PIN);
        int rawB = analogRead(PC2_PIN);
        float vC = rawC * (5.0f / 1023.0f) - 2.375f;
        float vB = rawB * (5.0f / 1023.0f) - 2.375f;
        float I_B = (vB / 100.0f) / 0.004f;
        float I_C = (vC / 100.0f) / 0.004f;

        printf("RailB (i-sns-B): V=%.3f  I=%.3f A\n", vB, I_B);
        printf("RailC (i-sns-C): V=%.3f  I=%.3f A\n", vC, I_C);

        // Rail voltages via DG4051 on A6
        bool mainFault = false;     // Sense_0, Sense_2, Sense_3, Sense_5
        bool sense1Fault = false;   // Sense_1 only

        for (int i = 0; i < NUM_RAILS; i++) {
            RailSpec* r = &rails[i];
            selectMuxChannel(r->channel);
            int raw = readADC(MUX_OUT_PIN);
            float v = adcToVolts(raw);
            float diff = v - r->expectedVolts;
            if (diff < 0) diff = -diff;
            bool ok = diff <= r->tolerance;

            if (!ok) {
                if (strcmp(r->name, "Sense_1") == 0) sense1Fault = true;
                else mainFault = true;  // Sense_0 / 2 / 3 / 5 → fatal
            }

            // Map to friendly names
            const char* friendlyName = r->name;
            if (strcmp(r->name, "Sense_0") == 0) friendlyName = "V-sns-24V";
            else if (strcmp(r->name, "Sense_1") == 0) friendlyName = "V-sns-boost";
            else if (strcmp(r->name, "Sense_2") == 0) friendlyName = "V-sns-12V";
            else if (strcmp(r->name, "Sense_3") == 0) friendlyName = "V-sns-2.5V";
            else if (strcmp(r->name, "Sense_5") == 0) friendlyName = "V-sns-5V";

            printf("%s: %.3f V  (exp %.3f ±%.3f) -> %s\n",
                friendlyName, v, r->expectedVolts, r->tolerance,
                ok ? "OK" : "FAULT");
        }

        if (mainFault) {
            printf("Rails overall: UNSAFE\n");
        }
        else if (sense1Fault) {
            printf("Rails overall: SAFE (warning: V-sns-boost out of range)\n");
        }
        else {
            printf("Rails overall: SAFE\n");
        }

        // Temperature via MCP9700A on mux ch6
        int rawT = readMuxTemp();
        float tC = tempC_fromRaw(rawT);
        float tF = tempF_fromRaw(rawT);
        printf("Temp: %.2f °C  %.2f °F\n", tC, tF);

        printf("===========================================\n\n");
    }
}

int main(void) {
    cli();
    uart_init();
    stdout = &uart_output;
    adc_init();
    timer0_init();
    sei();

    setup();

    while (1) {
        loop();
    }
}
