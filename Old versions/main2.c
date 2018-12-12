/*
 * GccApplication1.c
 *
 * Created: 11/15/2018 8:07:42 AM
 * Author : ucrcse
 */ 

#include <avr/io.h>
#include "usart.h"
#include "lcd.h"
#include <math.h>

#define MQ135_DEFAULTPPM 392 //default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 41763 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2

double d = 0;
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

/*
 * get the calibrated ro based upon read resistance, and a know ppm
 */
long mq135_getro(long resvalue, double ppm) {
	return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR/ppm) / MQ135_EXPONENT ));
}

/*
 * get the ppm concentration
 */
double mq135_getppm(long resvalue, long ro) {
	double ret = 0;
	double validinterval = 0;
	validinterval = resvalue/(double)ro;
	if(validinterval<MQ135_MAXRSRO && validinterval>MQ135_MINRSRO) {
		ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/ro), MQ135_EXPONENT);
	}
	return ret;
}

int main(void)
{
	// LCD Control
	DDRC = 0xFF; PORTC = 0x00; // LCD data lines
	DDRD = 0xFF; PORTD = 0x00; // LCD control lines
	// Gas Sensors
	DDRB = 0x00; PORTB = 0xFF; // PORTB input
	// LED Alert
	DDRD = 0xFF; PORTD = 0x00; // PORTD output
	PORTD = 0x80;
	
	char Data_in;
	
	// Initialize USART0
	initUSART(0);
	// Initializes the LCD display
	LCD_init();
	LCD_DisplayString(1, "Hello World");
	// Init ADC
	A2D_init();
	
	while(1)
	{
		if (USART_HasReceived(0))
		{
			Data_in = USART_Receive(0);
			if (Data_in == '1')
			{
				PORTD |= (0x01 << 7); // Turn on LED
				if(USART_IsSendReady(0)) 
				{
					USART_Send(Data_in, 0);
				}
				LCD_DisplayString(1, Data_in);
			} 
			else
			{
				PORTD &= ~(0x01 << 7); // Turn off LED
				if(USART_IsSendReady(0)) 
				{
					USART_Send(Data_in, 0);
				}
				LCD_DisplayString(1, "Hello World");
			}
		}
		
	}
}