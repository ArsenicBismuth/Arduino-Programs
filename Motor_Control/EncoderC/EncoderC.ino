/* The average rotary encoder has three pins, seen from front: A C B
   Clockwise rotation A(on)->B(on)->A(off)->B(off)
   CounterCW rotation B(on)->A(on)->B(off)->A(off)
   
   Rotary Enc Ref : https://playground.arduino.cc/Main/RotaryEncoders
   USART Ref      : https://appelsiini.net/2011/simple-usart-with-avr-libc/
                    https://www.avrfreaks.net/forum/stdio-setup-printf-and-pulling-my-hair-out
*/

#define F_CPU 16000000UL
#define BAUD 9600
#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdio.h>

volatile unsigned long encPos = 0;  // a counter for the dial
unsigned long lastReportedPos = 1;  // change management
static boolean rotating = false;    // debounce management
float ang = 0.0;
float spd = 0.0;

unsigned long m = 0;                // Current time
unsigned long cmillis = 0;          // Current time
unsigned long pmillis = 0;          // Previous time

static FILE uart_output;
static FILE uart_input;

// usually the rotary encoders three pins have the ground pin in the middle
enum PinAssignments {
    encPinA = 2,   // right
    encPinB = 3   // left
};

// interrupt service routine vars
char A_set = 0;
char B_set = 0;

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

// Interrupt on A changing state
ISR(INT0_vect) {
    // debounce
    if ( rotating ) delay (1);  // wait a little until the bouncing is done
    // Test transition, did things really change?
    if ( digitalRead(encPinA) != A_set ) { // debounce once more
        A_set = !A_set;
        if ( A_set && !B_set ) encPos += 1; // adjust counter + if A leads B
        rotating = 0;  // no more debouncing until loop() hits again
    }
}

// Interrupt on B changing state, same as A above
ISR(INT1_vect) {
    if ( rotating ) delay (1);
    if ( digitalRead(encPinB) != B_set ) {
        B_set = !B_set;
        if ( B_set && !A_set ) encPos -= 1; //  adjust counter - 1 if B leads A
        rotating = 0;
    }
}

// Timer
ISR(TIMER2_OVF_vect) {
    TCNT2 = 5-1;                    // Re-init value for 1ms interrupt
    m++;
}

int main(void) {
    // Timer initialization
    TCCR2B = (1<<CS01|1<<CS02);     // Clock Select 1, prescaler set as 1/1024. // Overflow, set Timer/Counter mode to OVF
    TCNT2 = 5-1;                    // Initial value for 1ms interrupt
    TIMSK2 = 1<<TOIE2;              // Timer2 Overflow Interrupt Enable
    
    // Encoder
    DDRD &= ~(1 << encPinA) & ~(1 << encPinB);  // Clear the pins, turning into input
    PORTD |= (1 << encPinA) | (1 << encPinB);   // turn on the Pull-up

    EICRA |= (1 << ISC00) | (1 << ISC01);       // Set the interrupts to trigger on ANY logic change
    EIMSK |= (1 << INT0) | (1 << INT1) ;        // Turns on INT0 & INT1

    sei();                                      // turn on interrupts

    // Serial
    uart_init();
    
    fdev_setup_stream(&uart_output, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    fdev_setup_stream(&uart_input, NULL, uart_getchar, _FDEV_SETUP_READ);
    
    stdout = &uart_output;
    stdin  = &uart_input;
    
    printf("Angle(n) \tAngle(deg) \tSpeed(RPM)\n");
    
    // main loop, work is done by interrupt service routines, this one only prints stuff
    while (1) {
        cmillis = m;
        rotating = 1;  // reset the debouncer

        printf("%d\t", encPos);
        printf("%.2f\t", ang);
        printf("%d\n", spd);
            
        if (lastReportedPos != encPos) {
            ang = encPos * 0.15 * 4;    // Max 9830.25 Min 0.15 (degrees)
            spd = (encPos - lastReportedPos) * 1000 * 0.15 * 4/ (cmillis - pmillis) * 360 / 60;
            lastReportedPos = encPos;
            pmillis = cmillis;
        } else
            spd = 0;
    }
    return 1;
}

