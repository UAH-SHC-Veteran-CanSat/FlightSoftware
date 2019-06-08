/*
 * GPS.h
 *
 * Created: 6/7/2019 6:14:27 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>

#ifndef GPS_H_
#define GPS_H_

#define PACKET_NONE 0
#define PACKET_GGA 1
#define PACKET_OTHER 2

#define USART_GPS USARTC1
#define USART_GPS_RXC_vect USARTC1_RXC_vect

//MUST BE A FACTOR OF 2 AND <=128!
#define GPS_RX_BUFFER_SIZE 128

#define GPS_USART_SERIAL_BAUDRATE            9600 //Also called 9.6k Baud
#define GPS_USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc //Each packet of data is 8 bits long
#define GPS_USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc //No Parity, important in computer RAM, not for this
#define GPS_USART_SERIAL_STOP_BIT            true //One stop bit

void gps_init(double);

void gps_update();

double gps_get_longitude();
double gps_get_latitude();
double gps_get_altitude();
double gps_get_time();
uint16_t gps_get_sats();



#endif /* GPS_H_ */