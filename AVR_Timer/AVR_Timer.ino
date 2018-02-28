/*
 * Original: http://www.avr-tutorials.com/projects/atmega16-based-digital-clock
 * Further Developed by Dafa Faris Muhammad & Daniel Steven
 *
 * TinkerCAD: https://www.tinkercad.com/things/2AZw3X21r0k-copy-of-neat-snicket/editel?sharecode=ebuvlFIe8aLPWZyW79dfbpua2jYm88cSwJwrc6oba88=
 * Keypad Resistors: http://www.microchip.com/forums/m/tm.aspx?m=694613&p=1
 * ADC Reference: http://maxembedded.com/2011/06/the-adc-of-the-avr/
 */ 
 
#define F_CPU 16000000UL		// 16MHz, default in most Arduino

#include <stdint.h> 
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
 
#define SegCtrlPort	PORTC
#define SegCtrlPin		PINC
#define SegCtrlDDR		DDRC
 
// Should be (PORTB >> 2)
#define ButPort			PORTB
#define ButPin			PINB
#define ButDDR			DDRB
*/

/*Global Variables Declarations*/

/* 
* Port combiner, ex:
* PORTD[7:2] (porta) : PORTB[1:0] (portb) = SegData (multi) & 0x0F
* A SegData data equivalents to
*	SegData = [D7...D2, B1...B0]
*				= ((PORTD & 0b11111100) | (PORTB & 0b00000011));
* Thus, use below for applying
*	PORTD = (SegData & 0b11111100) | ~0b11111100;
*	PORTB = (SegData & 0b00000011) | ~0b11111100;
* Forcing the rest to high, keep pullup settings
*/

uint16_t SegData = 0;	// PORTD[7:2] & PORTB[1:0]
#define SegDataa PORTD
#define SegDatab PORTB
#define SegDataConf 0b11111100
uint16_t SegCtrl = 0;	// PORTB[7:5] & PORTC[4:0]
#define SegCtrla PORTB
#define SegCtrlb PORTC
#define SegCtrlConf 0b11100000

#define PINADC 5		// Use A6, only available in SMD ATmega
#define BUTTER 3		// Allowable ADC error from specified button mapping value

uint8_t hours = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t curs = 0;		// Specify cursor location, displayed using the period

int16_t intm = 0;		// Main integer to be displayed & operated on non-clock mode

uint16_t inta = 0;		// Integers ONLY to store calculator data
uint16_t intb = 0;
uint8_t sign = 0;		// Negative or not sign

uint8_t but = 0xFF;		// Button mapped from ADC reading
uint8_t pbut = 0;		// Previous button state
uint8_t mode = 0;		// Modes, total of 5: clock, set clock, calculation modes (+, -, *, /), enter
 
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
uint8_t DigitTo7SegEncoder(uint8_t digit, uint8_t common);

/*Update 7-segment display*/
void UpdateDisplay(uint8_t d5, uint8_t d4, uint8_t d3, uint8_t d2, uint8_t d1, uint8_t d0, uint8_t dot);

/*Update combined ports used by the 7-segments*/
void UpdateMultiPorts(uint16_t *multi, uint16_t *porta, uint16_t *portb, uint16_t confa,  uint16_t confb);

/*Read analog data using ADC*/
uint16_t AnalogRead(uint8_t ch);

/*Map Buttons from ADC Reading*/
uint8_t Button(uint8_t adc, uint8_t error);

/*Compare with error*/
uint8_t InRange(uint8_t ch);

/*Absolute*/
uint16_t Abs(int16_t in);
 
/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect);
 
 
/*Main Program*/
/*****************************************************************************/
int main(void)
{	
	//Serial.begin(9600);
	
	/* MCU parameter configs */
	// Pins initialization
	DDRD = SegDataConf;					// Output 7-segment data [7:2]
	DDRB = ~SegDataConf | SegCtrlConf;	// Output seg data [1:0], Input [5:2], Output seg control [7:6]
	DDRC = ~SegCtrlConf;				// Output seg common pin (control)
	
	PORTD = 0xFF;		// Enable pullup on input, set HIGH for output
	PORTB = 0xFF;
	PORTC = ~SegCtrlConf;
 
	// Timer initialization
	// TCNT1						// Background variable which counts system clock ticks after modified by Prescaler 1
	TCCR1B = (1<<CS12|1<<CS10|1<<WGM12);	// Clock Select 1, prescaler set as 1/1024. // Clear Time on Compare 1, set Timer/Counter mode to CTC
	OCR1A = 15625-1;				// Ouput Compare Match, value to be referenced when comparing
	TIMSK1 = 1<<OCIE1A;				// Timer/Counter1, Output Compare A Match Interrupt enable
	sei();							// Set global interrupt flag
	
	// ADC initialization
    ADMUX = (1<<REFS0);	// AREF = AVcc
 
    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
 
	while(1) {
		/*
		* Basic Operations
		* Buttons: Change modes (A-D), digits (0-9), erase (*), & clear (#)
		* - Buttons handled only on "Rising", preventing multiple strokes.
		* - Clock functions normally even while being used as a calculator.
		* Modes:
		* - [A] Clock		(mode 0)
		* - [B] Set Clock	(mode 1) Can't adjust the last two seconds due to int16 limitation
		* - [C] Calculator	(mode 2 input A, mode 3 input B, mode 4-7 various results)
		* - [D] Enter		(apply & increment calculator modes)
		*/
		
		/*Button & mode handling*/
		// Basic multi buttons management from single ADC pin
		but = Button(AnalogRead(PINADC), BUTTER);
		
		// Check rising, button isn't active while previously it's
		if((but == 255) && (pbut != 255)) {
			switch(pbut) {					// [Important] Read the previous button, not current
				// Mode handling buttons
				case 'a':	// Clock
					intm = 0;
					mode = 0;
					break;
				case 'b':	// Set clock
					intm = 0;
					mode = 1;
					break;
				case 'c':	// Calculator
					mode = 2;
					intm = 0;
					inta = 0;
					intb = 0;
					break;
				case 'd':	// Enter
					switch (mode) {
						case 1:	// Set clock
							/*seconds = intm%100;
							intm /= 100;*/
							minutes = intm%100;
							intm /= 100;
							hours = intm%100;
							intm = 0;
							mode = 0;
							break;
						case 2:	// Set first calculator number
							inta = intm;
							intm = 0;
							mode++;
							break;
						case 3:	// Set second calculator number
							intb = intm;
							intm = 0;
							mode++;
							break;
						case 4:
						case 5:
						case 6: mode++; break;	 // Go to the next calculation mode
						case 7: mode = 2; break; // Loop again, to review
						default: break;
					}
					break;
				// Non-mode buttons
				case '*': intm /= 10; break;	// Undo latest number input
				case '#': intm = 0; break;		// Clear
				// Normal input, 0-9. Limit input to 4 digits
				default: if (intm <= 999) intm = intm * 10 + pbut; break;
			}
		}
        pbut = but;	// Store button state
		
		//Serial.print(mode);Serial.print(' ');Serial.print(intm);Serial.println();
		
		/*Calculation handling*/
		switch(mode) {
			case 4: intm = inta + intb; break;	// Summation
			case 5: intm = (int) inta - (int) intb; break;	// Reduction
			case 6: intm = inta * intb; break;	// Multiplication
			case 7: intm = inta / intb; break;	// Division
			default: break;
		}
		
		/*Display handling*/
		
		switch (mode)
		{
			case 0: UpdateDisplay(hours/10, hours%10, minutes/10, minutes%10, seconds/10, seconds%10, 2); break;
			case 1: UpdateDisplay(intm/1000, intm%1000/100, intm%100/10, intm%10, seconds/10, seconds%10, 2); break;
			case 2: UpdateDisplay('a', '=', intm/1000%10, intm/100%10, intm/10%10, intm%10, curs); break;
			case 3: UpdateDisplay('b', '=', intm/1000%10, intm/100%10, intm/10%10, intm%10, curs); break;
			case 4: 
			case 5:
			case 6:
			case 7: 
				sign = (intm < 0)*'-' | (intm >= 0)*' ';
				intm = Abs(intm);
				UpdateDisplay(mode-3, sign, intm/1000%10, intm/100%10, intm/10%10, intm%10, curs); break;
			default: break;
		}
    }
	return 0;
}


/*
* Update 7-segment display
* Ordered from leftmost to rightmost.
* Set one 7-segments at a time for every loop,
* fast enough to appear as if they're all always on.
* Isolation done by the Control Ports
*/
void UpdateDisplay(uint8_t d5, uint8_t d4, uint8_t d3, uint8_t d2, uint8_t d1, uint8_t d0, uint8_t dot){
	uint8_t digits[6] = {d0, d1, d2, d3, d4, d5};
	for (int i = 0; i < 6; i++) {
		SegData = DigitTo7SegEncoder(digits[i], 1) | ((1<<7) * (dot == i));
		SegCtrl = ~(0x01<<i);
		UpdateMultiPorts(&SegData, (uint16_t*)&SegDataa, (uint16_t*)&SegDatab, SegDataConf,  ~SegDataConf);
		UpdateMultiPorts(&SegCtrl, (uint16_t*)&SegCtrla, (uint16_t*)&SegCtrlb, SegCtrlConf,  ~SegCtrlConf);
		_delay_ms(2);
	}
}

/* 
* Port combiner, ex:
* PORTD[7:2] (porta) : PORTB[1:0] (portb) = SegData (multi) & 0x0F
* Thus, a SegData data equivalents to
*	SegData = [D7...D2, B1...B0]
*				= ((PORTD & 0b11111100) | (PORTB & 0b00000011));
uint16_t SegData = 0;	// PORTD[7:2] & PORTB[1:0]
#define SegDataa PORTD
#define SegDatab PORTB
#define SegDataConf 0b11111100
uint16_t SegCtrl = 0;	// PORTB[7:5] & PORTC[4:0]
#define SegCtrla PORTB
#define SegCtrlb PORTC
#define SegCtrlConf 0b11100000
*/
void UpdateMultiPorts(uint16_t *multi, uint16_t *porta, uint16_t *portb, uint16_t confa,  uint16_t confb)
{
	// Get the corresponding port data from multi and keep the data unused the multi
	*porta = (*multi & confa) | (*porta & ~confa);
	*portb = (*multi & confb) | (*portb & ~confb);
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
uint8_t DigitTo7SegEncoder(uint8_t digit, uint8_t common)
{
	uint8_t SegVal;
 
	switch(digit)	
	{	
		case 0:	SegVal = 0b00111111; break;
		case 1:	SegVal = 0b00000110; break;
		case 2:	SegVal = 0b01011011; break;
		case 3:	SegVal = 0b01001111; break;
		case 4:	SegVal = 0b01100110; break;
		case 5:	SegVal = 0b01101101; break;
		case 6:	SegVal = 0b01111101; break;
		case 7:	SegVal = 0b00000111; break;
		case 8:	SegVal = 0b01111111; break;
		case 9:	SegVal = 0b01101111; break;
		case '=':SegVal = 0b01001000; break;
		case 'a':SegVal = 0b01110111; break;
		case 'b':SegVal = 0b01111100; break;
		case ' ':SegVal = 0b00000000; break;
		case '-':SegVal = 0b01000000; break;
	}		
	if (!common) SegVal = ~SegVal;
	
	return SegVal;
}
 
/*Timer Counter 1 Compare Match A Interrupt Service Routine/Interrupt Handler*/
ISR(TIMER1_COMPA_vect)
{
	seconds++;
 
	if(seconds >= 60) {
		seconds = 0;
		minutes++;
	}
	if(minutes >= 60) {
		minutes = 0;
		hours++;		
	}
	if(hours >= 24)
		hours = 0;
}

/*Read analog data using ADC*/
uint16_t AnalogRead(uint8_t ch)
{
  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
 
  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);
}

/*Map Buttons from ADC Reading*/
uint8_t Button(uint16_t adc, uint16_t error)
{
	uint8_t buts[16] = {  1,  2,  3,'a',  4,  5,  6,'b',  7,  8,  9,'c','*' ,0,'#','d'};
	uint16_t adcs[16] = {559,541,521,500,480,456,428,398,370,334,293,247,203,145, 78,  0};
	
	uint8_t i = 0;
	uint8_t button = 255;	// Default state, unpressed
	
	while((button == 255) && (i < 16)) {
		if (InRange(adc, adcs[i], error)) button = buts[i];
		i++;
	}
	
	//Serial.print(adc); Serial.print(' '); Serial.print(button); Serial.print(' ');
	
	return button;
}

/*Compare with error*/
uint8_t InRange(uint16_t in, uint16_t compare, uint16_t error)
{
	uint16_t max, min;
	
	// Simple overflow prevention,
	// but can only handle for compare absolute max or min
	if (compare == 0xFF) max = compare;
	else max = compare + error;
	if (compare == 0x00) min = compare;
	else min = compare - error;
	
	if ((in <= max) && (in >= min)) return 1;
	else return 0;
}

/*Absolute*/
uint16_t Abs(int16_t in)
{
	if (in >= 0) return in;
	else return -1*in;
}