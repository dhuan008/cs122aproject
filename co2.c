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
#include <math.h>

//char Data_in;
unsigned short adc_value;


#define MQ135_PULLDOWNRES 22000
#define ADC_REFRES 1024 //reference resolution used for conversions
#define MQ135_DEFAULTPPM 406 //default ppm of CO2 for calibration https://www.co2.earth/
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

void adc_init() {
	// AREF = AVcc
	ADMUX = (1<<REFS0);
	
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

short adc_read(short ch) {
	// select the corresponding channel 0~7
	// ANDing with 7? will always keep the value
	// of ch between 0 and 7
	ch &= 0b00000111; // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	// start single convertion
	// write 1? to ADSC
	ADCSRA |= (1<<ADSC);
	// wait for conversion to complete
	// ADSC becomes 0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	return (ADC);
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

// convert an adc value to a resistance value
long adc_getresistence(uint16_t adcread, uint16_t adcbalanceresistor)
{
	if(adcread == 0)
		return 0;
	else
		return (long)((long)(ADC_REFRES*(long)adcbalanceresistor)/adcread-(long)adcbalanceresistor);
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
	LCD_DisplayString(1, "Warming up");
	// Init ADC
	adc_init();
	// Init USART(0);
	initUSART(0);
	
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
		
		adc_value = adc_read(0);
		char horiz_buf[5];			// buffer to store the ascii value of adcchar horiz_buf[5];			// buffer to store the ascii value of adc
		itoa10(adc_value, horiz_buf);		// copy over ascii value of short int and store in buffer
		
		// Calculated resistance depends on the sensor pulldown resistor
		long res = adc_getresistence(adc_value, MQ135_PULLDOWNRES);
		mq135_ro = mq135_getro(res, MQ135_DEFAULTPPM);
		d = mq135_getppm(res, MQ135_DEFAULTRO);
		d = d * 1000;
		dtostrf(d, 3, 0, horiz_buf);
		

 		output1 = concat("CO2", "             ");
 		output2 = concat("PPM: ", horiz_buf);
 		output1 = concat(output1, output2);
		
		LCD_DisplayString(1, output1);
	}
	
	
}