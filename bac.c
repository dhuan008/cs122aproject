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

// ADC
unsigned short adc_value;

// CO2 Constants
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

// Alcohol Constants
#define SENSOR_PIN  A1
#define BRIGHTNESS  80
#define S7_ADDR     0x71
#define DEC_MASK    0b00000001
#define BAC_START   410     // Beginning ADC value of BAC chart
#define BAC_END     859     // Lsat ADC value in BAC chart
const uint8_t bac_chart[] = {2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 16,
	16, 16, 17, 17, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22,
	23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26,
	26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28,
	28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29,
	29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29,
	29, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 31, 31, 31,
	31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 33,
	33, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35,
	35, 35, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38,
	38, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 42,
	42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 45, 45, 45, 45,
	45, 46, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 48, 48, 48, 48, 48, 48, 49, 49,
	49, 49, 49, 49, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52,
	53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 56, 56, 56, 56,
	56, 57, 57, 57, 57, 58, 58, 58, 58, 58, 59, 59, 59, 59, 60, 60, 60, 60, 61, 61,
	61, 61, 62, 62, 62, 62, 63, 63, 63, 64, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 
	67, 68, 68, 68, 69, 69, 69, 70, 70, 71, 71, 72, 72, 72, 73, 73, 74, 74, 75, 75, 
	76, 76, 77, 78, 78, 79, 79, 80, 80, 81, 82, 82, 83, 84, 84, 85, 86, 87, 87, 88, 
	89, 90, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 
	106, 107, 108, 109, 111, 112, 113, 114, 116, 117, 118, 120, 121, 122, 124, 125, 
	127, 128, 130, 131, 133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 153, 
	155, 157, 159, 161, 163, 165, 168, 170, 172, 174, 177, 179, 182, 184, 187, 189, 192, 194};
// Alcohol Globals
uint8_t bac;

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
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111; // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	// wait for conversion to complete
	// ADSC becomes ’0? again
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

// CO2 get the ppm concentration
double mq135_getppm(long resvalue, long ro) {
	double ret = 0;
	double validinterval = 0;
	validinterval = resvalue/(double)ro;
	ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/ro), MQ135_EXPONENT);
	return ret;
}

// CO2 convert an adc value to a resistance value
long adc_getresistence(uint16_t adcread, uint16_t adcbalanceresistor)
{
	if(adcread == 0)
		return 0;
	else
		return (long)((long)(ADC_REFRES*(long)adcbalanceresistor)/adcread-(long)adcbalanceresistor);
}

char horiz_buf[5];			// buffer to store the ascii value of adcchar horiz_buf[5];			// buffer to store the ascii value of adc
enum Sensor_States {CO2, Combust, Alcohol} Sensor_State;
void Sensor_Tick()
{
	// Transitions
	switch (Sensor_State)
	{
		case CO2:
			if (USART_HasReceived(0))
			{
				USART_Flush(0);
				Sensor_State = Combust;
			}
			else
			{
				Sensor_State = CO2;
			}
			break;
		case Combust:
			if (USART_HasReceived(0))
			{
				USART_Flush(0);
				Sensor_State = Alcohol;
			}
			else
			{
				Sensor_State = Combust;
			}
			break;
		case Alcohol:
			if (USART_HasReceived(0))
			{
				USART_Flush(0);
				Sensor_State = CO2;
			}
			else
			{
				Sensor_State = Alcohol;
			}
			break;
		default:
			Sensor_State = CO2;
			break;
	}
	// Actions
	switch (Sensor_State)
	{
		case CO2:
			adc_value = adc_read(0);
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
			
			break;
		case Combust:
			adc_value = adc_read(0);
			itoa10(adc_value, horiz_buf);		// copy over ascii value of short int and store in buffer
			
			output1 = concat("Butane", "          ");
			output2 = concat("PPM: ", horiz_buf);
			output1 = concat(output1, output2);
			
			LCD_DisplayString(1, output1);
			break;
		case Alcohol:
			adc_value = adc_read(1);
			itoa10(adc_value, horiz_buf);		// copy over ascii value of short int and store in buffer
			
			// Calculate ppm. Regression fitting from MQ-3 datasheet.
			// Equation using 5V max ADC and RL = 4.7k. "v" is voltage.
			// PPM = 150.4351049*v^5 - 2244.75988*v^4 + 13308.5139*v^3 -
			//       39136.08594*v^2 + 57082.6258*v - 32982.05333
			// Calculate BAC. See BAC/ppm chart from page 2 of:
			// http://sgx.cdistore.com/datasheets/sgx/AN4-Using-MiCS-Sensors-for-Alcohol-Detection1.pdf
			if (ADC < BAC_START)
			{
				bac = 0;
				sprintf(horiz_buf, "0000");
			}
			else if (horiz_buf > BAC_END)
			{
				sprintf(horiz_buf, "EEEE");
			}
			else
			{
				adc_value = adc_value - BAC_START;
				bac = bac_chart[adc_value];
				if (bac < 10)
				{
					sprintf(horiz_buf, "000%1d", bac);
				}
				else if (bac < 100)
				{
					sprintf(horiz_buf, "00%2d", bac);
				}
				else
				{
					sprintf(horiz_buf,"0%3d", bac);
				}
			}
			
			output1 = concat("Alcohol", "         ");
			output2 = concat("BAC: ", horiz_buf);
			output1 = concat(output1, output2);
			
			LCD_DisplayString(1, output1);
			break;
	}
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
		
		Sensor_Tick();
		
	}
	
	
}