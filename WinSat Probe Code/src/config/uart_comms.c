/*
 * usart_comms.c
 *
 * Created: 8/18/2015 12:16:38 PM
 *  Author: abower
 */ 

/* This driver contains functions that can be called during initialization of the MCU
   to start the uart communications */

#include "conf_usart_serial.h" //Includes the information in the config file
#include "usart_command.h"


void UART_Comms_Init(void){
	//Struct that contains the settings from config file
	static usart_serial_options_t usart_options = {
		.baudrate = USART_SERIAL_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};

	//Must set TX pin as output
	PORTC.DIRSET=0b00001000;

	stdio_serial_init(&USARTC0, &usart_options); //ASF function that initializes the UART peripheral
	usart_set_rx_interrupt_level(&USARTC0, USART_INT_LVL_HI);
}