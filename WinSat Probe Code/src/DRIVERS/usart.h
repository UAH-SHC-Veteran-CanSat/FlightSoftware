/*
 * usart.h
 *
 * Created: 4/8/2019 2:56:40 AM
 *  Author: Mason
 */ 


#ifndef USART_H_
#define USART_H_

#include <asf.h>
#include <string.h>
#include <ctype.h>


#define USART_XBEE USARTD0
#define USART_OPENLOG USARTC0

//Whichever device is uncommented below will respond to printf, the other will work with log_printf
#define XBEE_PRIMARY
//#define OPENLOG_PRIMARY

//These #defines are the "settings" for the USART communication
//The receiving end, computer or radio, must have the same settings or it won't work
#define USART_SERIAL_BAUDRATE            115200 //Also called 115k Baud
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc //Each packet of data is 8 bits long
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc //No Parity, important in computer RAM, not for this
#define USART_SERIAL_STOP_BIT            true //One stop bit


void UART_Comms_Init(void); //This will initialize the uart communications

void log_printf(const char*, ...);


#endif /* USART_H_ */