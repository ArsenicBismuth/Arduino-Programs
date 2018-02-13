/*
 * Original: http://www.avr-tutorials.com/projects/atmega16-based-digital-clock
 * DigitalClock.c
 * Written in AVR Studio 5
 * Compiler: AVR GNU C Compiler (GCC)
 *
 * Created: 12/10/2011 1:17:19 PM
 * Author:	AVR Tutorials
 * Website:	www.AVR-Tutorials.com
 */ 
 
#define F_CPU 4000000UL		// 4 MHz

#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
#define SegDataPort		PORTB
#define SegDataPin		PINB
#define SegDataDDR		DDRB
 
#define SegCntrlPort	PORTC
#define SegCntrlPin		PINC
#define SegCntrlDDR		DDRC
 
 
/*Global Variables Declarations*/
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
 
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
 
/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect);
 
 
/*Main Program*/
/*****************************************************************************/
int main(void)
{
	// MCU parameter configs
    SegDataDDR = 0xFF;
	SegCntrlDDR = 0x3F;
	SegCntrlPort = 0xFF;
 
	TCCR1B = (1<<CS12|1<<WGM12);	// Clock Select 1, prescaler set as 1/256. // Clear Time on Compare 1, set Timer/Counter mode to CTC
	OCR1A = 15625-1;				// Ouput Compare Match, value to be referenced when comparing
	TIMSK1 = 1<<OCIEA;				// Timer/Counter1, Output Compare A Match Interrupt enable
	sei();							// Set global interrupt flag
 
 
	while(1)
    {
        /* Set Minutes when SegCntrl Pin 6 Switch is Pressed*/
		if((SegCntrlPin & 0x40) == 0 )
		{	
			_delay_ms(200);
			if(minutes < 59)
				minutes++;
			else
				minutes = 0;
		}
		
        /* Set Hours when SegCntrl Pin 7 Switch is Pressed */
		if((SegCntrlPin & 0x80) == 0 )
		{	
			_delay_ms(200);
			if(hours < 23)
				hours++;
			else
				hours = 0;
		}
		
		
		/* 
		* Set one 7-segments at a time for every loop,
		* fast enough to appear as if they're all always on.
		* Isolation done by the Control Ports
		*/
		SegDataPort = DigitTo7SegEncoder(seconds%10,1);
		SegCntrlPort = ~0x01;
		SegDataPort = DigitTo7SegEncoder(seconds/10,1); 
		SegCntrlPort = ~0x02;
		SegDataPort = DigitTo7SegEncoder(minutes%10,1);
		SegCntrlPort = ~0x04;
		SegDataPort = DigitTo7SegEncoder(minutes/10,1); 
		SegCntrlPort = ~0x08;
		SegDataPort = DigitTo7SegEncoder(hours%10,1); 
		SegCntrlPort = ~0x10;
		SegDataPort = DigitTo7SegEncoder(hours/10,1);
		SegCntrlPort = ~0x20;
 
    }
	return 0;
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