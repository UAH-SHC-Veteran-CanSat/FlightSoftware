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

#define USART_XBEE USARTD1
#define USART_XBEE_RXC_vect USARTD1_RXC_vect

#define USART_OPENLOG USARTC0
#define USART_OPENLOG_RXC_vect USARTC0_RXC_vect


//Whichever device is uncommented below will respond to printf, the other will work with log_printf
#define XBEE_PRIMARY
//#define OPENLOG_PRIMARY

#define SECONDARY_TX_BUFFER_SIZE 128

//MUST BE A FACTOR OF 2 AND <=128!
#define PRIMARY_RX_BUFFER_SIZE 128

//These #defines are the "settings" for the USART communication
//The receiving end, computer or radio, must have the same settings or it won't work
#define USART_SERIAL_BAUDRATE            115200 //Also called 115.2k Baud
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc //Each packet of data is 8 bits long
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc //No Parity, important in computer RAM, not for this
#define USART_SERIAL_STOP_BIT            true //One stop bit


void UART_Comms_Init(void); //This will initialize the uart communications

void log_printf(const char*, ...);

bool is_command_ready();

char* get_command();


#endif /* USART_H_ */