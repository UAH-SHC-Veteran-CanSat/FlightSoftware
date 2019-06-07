/*
 * uart.c
 *
 * Created: 4/8/2019 2:56:11 AM
 *  Author: Mason
 */ 
#include "usart.h"


void UART_Comms_Init(void){
	
	//Initializes XBee
	static usart_serial_options_t usart_options = {
		.baudrate = USART_SERIAL_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	
	//Initialize the OpenLogger
	static usart_serial_options_t openlogger_usart_options = {
		.baudrate = USART_SERIAL_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	//Sends bit over Xbee
	PORTD.DIRSET = 0b00001000;
	PORTD.DIRCLR = 0b00000100;
	
	//Sends bit over Openlogger
	PORTC.DIRSET = 0b00001000;
	
#ifdef XBEE_PRIMARY

	stdio_serial_init(&USART_XBEE, &usart_options);
	usart_serial_init(&USART_OPENLOG, &openlogger_usart_options);
	
	usart_set_rx_interrupt_level(&USART_XBEE, USART_INT_LVL_HI);
	
#endif 
#ifdef OPENLOG_PRIMARY
	
	stdio_serial_init(&USART_OPENLOG, &usart_options);
	usart_serial_init(&USART_XBEE, &openlogger_usart_options);
	
	usart_set_rx_interrupt_level(&USART_OPENLOG, USART_INT_LVL_HI);
	
#endif
}

void log_printf(const char* format, ...)
{
	va_list vargs;
	char buffer[SECONDARY_BUFFER_SIZE];
	
	if(strlen(format) > SECONDARY_BUFFER_SIZE) {
		printf("logged format string is too long (>%i chars)\n", SECONDARY_BUFFER_SIZE);
	}
	
	va_start(vargs, format);
	vsprintf(buffer, format, vargs);
	va_end(vargs);
	
	uint16_t length = strlen(buffer);
	
#ifdef XBEE_PRIMARY
	
	usart_serial_write_packet(&USART_OPENLOG, buffer, length);

#endif
#ifdef OPENLOG_PRIMARY

	usart_serial_write_packet(&USART_XBEE, buffer, length);

#endif
	
}