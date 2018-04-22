/*
   USART Ref:   https://appelsiini.net/2011/simple-usart-with-avr-libc/
                https://www.avrfreaks.net/forum/stdio-setup-printf-and-pulling-my-hair-out
*/

#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdio.h>

static FILE uart_output;
static FILE uart_input;

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
    UCSR0A |= _BV(U2X0);
    #else
    UCSR0A &= ~(_BV(U2X0));
    #endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX 
}

int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

// Int because fdev requirement. Function must be received as "char".
// Just do: char a = uart_getchar();
int uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0); // Wait until data exists. 
    return UDR0;
}

int main(void) {
    // Serial
    uart_init();
    
    fdev_setup_stream(&uart_output, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    fdev_setup_stream(&uart_input, NULL, uart_getchar, _FDEV_SETUP_READ);
    
    stdout = &uart_output;
    stdin  = &uart_input;

    char input;
    
    // main loop, work is done by interrupt service routines, this one only prints stuff
    while (1) {
        puts("Hello world!");
        input = getchar();
        printf("You wrote %c\n", input);
    }
    return 1;
}

