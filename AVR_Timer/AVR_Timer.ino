/*
 * Original: http://www.avr-tutorials.com/projects/atmega16-based-digital-clock
 *
 */ 
 
#define F_CPU 16000000UL		// 16MHz, default in most Arduino

#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
// PORT	= Output buffer
// PIN	= Input buffer

// Should be ((PORTB << (16-2)) | (PORTD >> 2)),
// but it's illegal for data assigment since they're
// two separate variables
/*
#define SegDataPort		PORTD
#define SegDataPin		PIND
#define SegDataDDR		DDRD
 
#define SegCntrlPort	PORTC
#define SegCntrlPin		PINC
#define SegCntrlDDR		DDRC
 
// Should be (PORTB >> 2)
#define ButPort			PORTB
#define ButPin			PINB
#define ButDDR			DDRB
*/

// For data reading, below is legal
#define ButPin			(PINB >> 2)
 
/*Global Variables Declarations*/
unsigned char SegDataPort = 0;
/* 
* Port combiner
* Combine PORTD[7:2] & PORTB[1:0]
*	SegDataPort = [D7...D2, B1...B0]
*				= ((PORTD & 0b11111100) | (PORTB & 0b00000011));
* Thus, use below for applying
*	PORTD = (SegDataPort & 0b11111100);
*	PORTB = (SegDataPort & 0b00000011);
*/

unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;

unsigned char int1 = 0;		// Leftmost number, to be operated against
unsigned char int2 = 0;
unsigned char result = 0;
unsigned char cursor = 0;	// Specify cursor location, displayed using the period

unsigned char pbut = 0;		// Previous button state
unsigned char calc = 0;		// Calculator modes, total of 5: inactive, sum, reduce, multi, div
 
/*
* Basic timer concepts
* Method	: Clear Time on Compare (CTC)
* Timer		: 01 - 16bit
*
* CTC is implemented in hardware, thus comparison is separate process.
* On the background Timer/Counter 1 (TCNT1) counts system clock ticks, which is modified by Prescaler 1.
* Whenever a match occurs (TCNT1 == Output Compare Match 1A, OCR1A), an interrupt is fired
* (because OCIEA is set) and the TIMER_COMPA_vect (OCF1A) flag is set. Executing the ISR clears the 
* clears that flag bit and automatically resets the TCNT1.
*/
 
/*Function Declarations*/
/*****************************************************************************/
/*Decimal Digit (0-9) to Seven Segment Values Encoder*/
unsigned char DigitTo7SegEncoder(unsigned char digit, unsigned char common);

/*Increment any of six digits based on location*/
unsigned char Increment(unsigned char digit);

/*Update combined ports used by the 7-segments*/
unsigned char UpdateMultiPorts();
 
/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect);
 
 
/*Main Program*/
/*****************************************************************************/
int main(void)
{
	// MCU parameter configs
	DDRD = 0b11111100;	// Output 7-segment data [7:2]
	DDRB = 0b00000011;	// Output 7-segment data [1:0], Input [7:2]
	
	PORTD = 0x00;		// Enable pullup on input, set LOW for output
	PORTB = 0x00;		// Enable pullup on input, set LOW for output
	
	DDRC = 0xFF;		// Output 7-segment common pin (control)
	PORTC = 0xFF;		// HIGH 7-segment common pin
 
	// TCNT1						// Background variable which counts system clock ticks after modified by Prescaler 1
	TCCR1B = (1<<CS12|1<<CS10|1<<WGM12);	// Clock Select 1, prescaler set as 1/1024. // Clear Time on Compare 1, set Timer/Counter mode to CTC
	OCR1A = 15625-1;				// Ouput Compare Match, value to be referenced when comparing
	TIMSK1 = 1<<OCIE1A;				// Timer/Counter1, Output Compare A Match Interrupt enable
	sei();							// Set global interrupt flag
 
	while(1) {
		/*
		* Basic Operations
		* Buttons: [Move cursor] [Increment digit] [Cycle modes]
		* - Buttons handled only on "Rising", preventing multiple strokes.
		* - Clock functions normally even while being used as a calculator.
		* - Every calculation works on-the-fly, any change of state will
		* directly reflected on result
		* - Cursor functionality allows for faster number manipulation,
		* digit selected by cursor (marked by a dot) is the increment target
		* instead of the number as a whole.
		*/
		
		// Basic multi buttons management
		for(unsigned char i = 0; i < 3; i++) {
			
			// Check rising, button isn't active while previously it's
			if(!(~ButPin && (1<<i)) && (~pbut && (1<<i))) {
				
				switch(i) {
					case 0:	// Cycle between multiple modes
						if(calc < 3) calc++;
						else calc = 0;
						break;
					case 1:	// Increment based on cursor position
						Increment(cursor);
						break;
					case 2:	// Move cursor
						if (cursor < 5) cursor++;
						else cursor = 0;
						break;
					default:
						break;
				}
			}

		}
		
        pbut = ButPin;	// Store button state
		
		// Calculate result on-the-fly
		switch(calc) {
			case 1: result = int1 + int2; break;	// Summation
			case 2: result = int1 - int2; break;	// Reduction
			case 3: result = int1 * int2; break;	// Multiplication
			case 4: result = int1 / int2; break;	// Division
			default: result = 0; break;
		}
		
		if(!calc) {
			/* 
			* Set one 7-segments at a time for every loop,
			* fast enough to appear as if they're all always on.
			* Isolation done by the Control Ports
			*/
			SegDataPort = DigitTo7SegEncoder(seconds%10,1) | ((1<<8) && (cursor == 0));
			UpdateMultiPorts();
			PORTC = ~0x01;
			SegDataPort = DigitTo7SegEncoder(seconds/10,1) | ((1<<8) && (cursor == 1)); 
			UpdateMultiPorts();
			PORTC = ~0x02;
			SegDataPort = DigitTo7SegEncoder(minutes%10,1) | ((1<<8) && (cursor == 2));
			UpdateMultiPorts();
			PORTC = ~0x04;
			SegDataPort = DigitTo7SegEncoder(minutes/10,1) | ((1<<8) && (cursor == 3)); 
			UpdateMultiPorts();
			PORTC = ~0x08;
			SegDataPort = DigitTo7SegEncoder(hours%10,1) | ((1<<8) && (cursor == 4)); 
			UpdateMultiPorts();
			PORTC = ~0x10;
			SegDataPort = DigitTo7SegEncoder(hours/10,1) | ((1<<8) && (cursor == 5));
			UpdateMultiPorts();
			PORTC = ~0x20;
		} else {
			SegDataPort = DigitTo7SegEncoder(result%10,1) | ((1<<8) && (cursor == 0));
			UpdateMultiPorts();
			PORTC = ~0x01;
			SegDataPort = DigitTo7SegEncoder(result/10,1) | ((1<<8) && (cursor == 1)); 
			UpdateMultiPorts();
			PORTC = ~0x02;
			SegDataPort = DigitTo7SegEncoder(int2%10,1) | ((1<<8) && (cursor == 2));
			UpdateMultiPorts();
			PORTC = ~0x04;
			SegDataPort = DigitTo7SegEncoder(int2/10,1) | ((1<<8) && (cursor == 3)); 
			UpdateMultiPorts();
			PORTC = ~0x08;
			SegDataPort = DigitTo7SegEncoder(int1%10,1) | ((1<<8) && (cursor == 4)); 
			UpdateMultiPorts();
			PORTC = ~0x10;
			SegDataPort = DigitTo7SegEncoder(int1/10,1) | ((1<<8) && (cursor == 5));
			UpdateMultiPorts();
			PORTC = ~0x20;
		}
 
    }
	return 0;
}

/* 
* Port combiner
* Combine PORTD[7:2] & PORTB[1:0]
*	SegDataPort = [D7...D2, B1...B0]
*				= ((PORTD & 0b11111100) | (PORTB & 0b00000011));
*/
unsigned char UpdateMultiPorts()
{
	PORTD = (SegDataPort & 0b11111100);
	PORTB = (SegDataPort & 0b00000011);
}
 
/*
* Encode a Decimal Digit 0-9 to its Seven Segment Equivalent.
* digit - Decimal Digit to be Encoded
* common - Common Anode (0), Common Cathode(1)
* SegVal - Encoded Seven Segment Value 
*
* Connections:
* Encoded SegVal is return in the order G-F-E-D-C-B-A with A as the least
* significant bit (bit 0) and G bit 6.
*/
unsigned char DigitTo7SegEncoder(unsigned char digit, unsigned char common)
{
	unsigned char SegVal;
 
	switch(digit)	
	{	
		case 0:	if(common == 1)	SegVal = 0b00111111;
				else			SegVal = ~0b00111111;
				break;
		case 1:	if(common == 1)	SegVal = 0b00000110;
				else			SegVal = ~0b00000110;
				break;
		case 2:	if(common == 1)	SegVal = 0b01011011;
				else			SegVal = ~0b01011011;
				break;
		case 3:	if(common == 1)	SegVal = 0b01001111;
				else			SegVal = ~0b01001111;
				break;
		case 4:	if(common == 1)	SegVal = 0b01100110;
				else			SegVal = ~0b01100110;
				break;
		case 5:	if(common == 1)	SegVal = 0b01101101;
				else			SegVal = ~0b01101101;
				break;
		case 6:	if(common == 1)	SegVal = 0b01111101;
				else			SegVal = ~0b01111101;
				break;
		case 7:	if(common == 1)	SegVal = 0b00000111;
				else			SegVal = ~0b00000111;
				break;
		case 8:	if(common == 1)	SegVal = 0b01111111;
				else			SegVal = ~0b01111111;
				break;
		case 9:	if(common == 1)	SegVal = 0b01101111;
				else			SegVal = ~0b01101111;		
	}		
	return SegVal;
}

unsigned char Increment(unsigned char digit)
{
	if(!calc) {
		switch(digit) {
			case 0:
				if (seconds % 10 < 9) seconds++;
				else seconds = (seconds / 10) * 10;
				break;
			case 1:
				if (seconds + 10 < 60) seconds += 10;
				else seconds %= 10;
				break;
			case 2:
				if (minutes % 10 < 9) minutes++;
				else minutes = (minutes / 10) * 10;
				break;
			case 3:
				if (minutes + 10 < 60) minutes += 10;
				else minutes %= 10;
				break;
			case 4:
				if (hours % 10 < 9) hours++;
				else hours = (hours / 10) * 10;
				break;
			case 5:
				if (hours + 10 < 24) hours += 10;
				else hours %= 10;
				break;
		}
	} else {
		switch(digit) {
			case 0: break;	// Can't change result
			case 1: break;
			case 2:
				if (int2 % 10 < 9) int2++;
				else int2 = (int2 / 10) * 10;
				break;
			case 3:
				if (int2 + 10 < 100) int2 += 10;
				else int2 %= 10;
				break;
			case 4:
				if (int1 % 10 < 9) int1++;
				else int1 = (int1 / 10) * 10;
				break;
			case 5:
				if (int1 + 10 < 100) int1 += 10;
				else int1 %= 10;
				break;
		}
	}
}
 
/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect)
{
	seconds++;
 
	if(seconds == 60) {
		seconds = 0;
		minutes++;
	}
	if(minutes == 60) {
		minutes = 0;
		hours++;		
	}
	if(hours > 23)
		hours = 0;
}
