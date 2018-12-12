/*
 * GccApplication1.c
 *
 * Created: 11/15/2018 8:07:42 AM
 * Author : ucrcse
 */ 

#include <avr/io.h>
#include "usart.h"
#include "lcd.h"


int main(void)
{
	// LCD Control
	DDRA = 0xFF; PORTA = 0x00; // LCD data lines
	DDRC = 0xFF; PORTC = 0x00; // LCD control lines
	// Gas Sensors
	DDRB = 0x00; PORTB = 0xFF; // PORTB input
	// LED Alert
	DDRD = 0xFF; PORTD = 0x00; // PORTD output
	PORTD = 0xFF;
	
	char Data_in;
	
	// Initialize USART0
	initUSART(0);
	// Initializes the LCD display
	LCD_init();
	
	
	while(1)
	{
		if (USART_HasReceived(0))
		{
			Data_in = USART_Receive(0);
			if (Data_in == '1')
			{
				PORTB |= (1<<PB0);
				if(USART_IsSendReady(0)) 
				{
					USART_Send(Data_in, 0);
				}
				LCD_DisplayString(1, Data_in);
			} 
			else
			{
				PORTB |= ~(1<<PB0);
				if(USART_IsSendReady(0)) 
				{
					USART_Send(Data_in, 0);
				}
				LCD_DisplayString(1, PORTB);
			}
		}
		
	}
}


