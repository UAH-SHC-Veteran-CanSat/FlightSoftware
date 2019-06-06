/*
 * uart.c
 *
 * Created: 4/8/2019 2:56:11 AM
 *  Author: Mason
 */ 
#include "asf.h"


void usartInit(void){
	
	//Initializes XBee
	static usart_serial_options_t usart_options = {
		.baudrate = 9600,
		.charlength = USART_CHSIZE_8BIT_gc,
		.paritytype = USART_PMODE_DISABLED_gc,
		.stopbits = true
	};
	
	
	//Initialize the OpenLogger
	static usart_serial_options_t openlogger_usart_options = {
		.baudrate = 9600,
		.charlength = USART_CHSIZE_8BIT_gc,
		.paritytype = USART_PMODE_DISABLED_gc,
		.stopbits = true
		
	};
	
	//Sends bit over Xbee
	PORTD.DIRSET = 0b00001000;
	PORTD.DIRCLR = 0b00000100;
	PORTD.OUTSET = 0b00001000;
	
	//Sends bit over Openlogger
	PORTC.DIRSET = 0b00001000;
	PORTC.OUTSET = 0b00001000;
	
	
	stdio_serial_init(&USARTD0, &usart_options);
	usart_serial_init(&USARTC0, &openlogger_usart_options);
	
	USARTD0.CTRLA = 0b00010000;
}