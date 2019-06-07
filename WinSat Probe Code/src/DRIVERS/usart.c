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
	
	
	//Initialize the GPS
	static usart_serial_options_t gps_usart_options = {
		.baudrate = 9600, //Hardcoded cause this is the only baudrate the GPS supports
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	//Sets pins for XBee
	PORTD.DIRSET = 0b10000000;
	PORTD.DIRCLR = 0b01000000;
	
	//Sets pins for OpenLog and GPS
	PORTC.DIRSET = 0b10001000;
	PORTC.DIRCLR = 0b01000100;
	
	
	
	
	usart_serial_init(&USART_GPS, &gps_usart_options);
	usart_set_rx_interrupt_level(&USART_GPS, USART_INT_LVL_HI);
	
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

ISR(USART_GPS_RXC_vect){
	char data;
	usart_serial_getchar(&USART_GPS, &data);
	printf("%c",data);
}

#ifdef OPENLOG_PRIMARY
	ISR(USART_OPENLOG_RXC_vect){
		char data;
		usart_serial_getchar(&USART_OPENLOG, &data);
		printf("%c",data);
	}
#endif

#ifdef XBEE_PRIMARY
ISR(USART_XBEE_RXC_vect){
	char data;
	usart_serial_getchar(&USART_XBEE, &data);
	printf("%c",data);
}
#endif