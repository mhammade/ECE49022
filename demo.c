#define F_CPU 16000000UL   // CPU clock speed (16 MHz)
#include <avr/io.h>
#include <util/delay.h>

// =============================
// UART INITIALIZATION
// =============================
void UART_init(unsigned int baud) {
    unsigned int ubrr = (F_CPU / (16UL * baud)) - 1;

    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0F = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0);     // Enable RX & TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 data bits, 1 stop bit
}

// =============================
// UART TRANSMIT
// =============================
void UART_sendChar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));   // Wait until buffer empty
    UDR0 = c;
}

// =============================
// UART PRINT STRING
// =============================
void UART_print(const char *str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}

// =============================
// UART RECEIVE
// =============================
char UART_readChar(void) {
    while (!(UCSR0A & (1 << RXC0)));   // Wait for incoming data
    return UDR0;
}

// =============================
// MAIN
// =============================
int main(void) {
    // Initialize UART at 9600 baud
    UART_init(9600);

    // Set PD6 (pin 6 on Arduino → OC0A → pin 12 on ATmega328P) as output
    DDRD |= (1 << DDD6);

    UART_print("Ready! Type 1=ON, 0=OFF\r\n");

    while (1) {
        if (UCSR0A & (1 << RXC0)) {    // If character received
            char c = UART_readChar();

            UART_print("Got: ");
            UART_sendChar(c);
            UART_print("\r\n");

            if (c == '1') {
                PORTD |= (1 << PORTD6);    // LED ON
                UART_print("LED ON\r\n");
            }

            if (c == '0') {
                PORTD &= ~(1 << PORTD6);   // LED OFF
                UART_print("LED OFF\r\n");
            }
        }
    }
}
