/*
 * GccApplication1.c
 *
 * Created: 11/15/2018 8:07:42 AM
 * Author : ucrcse
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "lcd.h"

//char Data_in;
unsigned short adc_value;
char horiz_buf[5];			// buffer to store the ascii value of adc

#define MQ135_DEFAULTPPM 392 //default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 41763 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2
long mq135_ro;
double d = 0;

/* USED FOR CREATING A SYNCRONOUS STATE MACHINE*/
volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

// ADEN: Enables analog-to-digital conversion
// ADSC: Starts analog-to-digital conversion
// ADATE: Enables auto-triggering, allowing for constant
//        analog to digital conversions.
// Note: Do not need to set the DDRA to enable the Analog to Digital circuitry
void A2D_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
}
unsigned short adc_read() {
	// start single conversion
	// write '1' to ADIF
	ADCSRA |= (1<<ADIF);
	
	// wait for conversion to complete
	// ADIF becomes '0' again
	while( ADCSRA & (1<<ADIF));
	return (ADC);
}

// Pins on PORTA are used as input for A2D conversion
// The default channel is 0 (PA0)
// The value of pinNum determines the pin on PORTA
// used for A2D conversion
// Valid values range between 0 and 7, where the value
// represents the desired pin for A2D conversion
void Set_A2D_Pin(unsigned char pinNum)
{
	ADMUX = (pinNum <= 0x07) ? pinNum : ADMUX;
	// Allow channel to stabilize
	static unsigned char i = 0;
	for ( i=0; i<15; i++ )
	{
		asm("nop");
	}
}

static char *itoa10_helper(int n, char *s) {
	if (n <= -10) {
		s = itoa10_helper(n / 10, s);
	}
	*s = (char) ('0' - n % 10);
	return ++s;
}

char* itoa10(int n, char *s) {
	if (n < 0) {
		*s = '-';
		*itoa10_helper(n, s+1) = 0;
		} else {
		*itoa10_helper(-n, s) = 0;
	}
	return s;
}

char* output1;
char* output2;
char* concat(const char *s1, const char *s2)
{
	const size_t str1len = strlen(s1);
	const size_t str2len = strlen(s2);
	char *result = malloc(str1len + str2len + 1); // Null terminator
	memcpy(result, s1, str1len);
	memcpy(result +str1len, s2, str2len +1); // Null terminator
	return result;
}


// get the calibrated ro based upon read resistance, and a know ppm
long mq135_getro(long resvalue, double ppm) {
	return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR/ppm) / MQ135_EXPONENT ));
}


// get the ppm concentration
double mq135_getppm(long resvalue, long ro) {
	double ret = 0;
	double validinterval = 0;
	validinterval = resvalue/(double)ro;
	ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/ro), MQ135_EXPONENT);
	return ret;
}




int main(void)
{
	DDRC = 0xFF; PORTC = 0x00; // LCD data lines
	DDRD = 0xFF; PORTD = 0x00; // LCD control lines
	// Gas Sensors
	DDRA = 0x00; PORTA = 0xFF; // PORTA input
	// Buttons
	DDRB = 0x00; PORTB = 0xFF; // PORTB input
	
	// Timer
	TimerSet(100);
	TimerOn();
	
	// Initializes the LCD display
	LCD_init();
	
	// Init ADC
	A2D_init();
	// Init USART(0);
	initUSART(0);
	
	// temp
	char Data_in;
	char printbuff[10];
	
	while(1) 
	{
		while(!TimerFlag);
		TimerFlag = 0;
		
		/*
		if (USART_HasReceived(0))
		{
			Data_in = USART_Receive(0);
			if (USART_IsSendReady(0))
			{
				USART_Send(Data_in, 0);
			}
			LCD_DisplayString(1, Data_in);
		}
		*/
		
		Set_A2D_Pin(0x00);
		adc_value = adc_read();
		itoa10(adc_value, horiz_buf);		// copy over ascii value of short int and store in buffer
		
		mq135_ro = mq135_getro(horiz_buf, MQ135_DEFAULTPPM);
		d = mq135_getppm(horiz_buf, MQ135_DEFAULTRO);
		dtostrf(d, 3, 5, printbuff);
		output1 = concat("CO2", "             ");
		output2 = concat("PPM: ", horiz_buf);
		output1 = concat(output1, output2);
		
		//LCD_DisplayString(17, "CO2");
		LCD_DisplayString(1, output1);
	}
	
	
}
