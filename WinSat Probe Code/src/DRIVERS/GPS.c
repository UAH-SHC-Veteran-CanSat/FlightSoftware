/*
 * GPS.c
 *
 * Created: 6/7/2019 6:14:06 PM
 *  Author: trbinsc
 */ 

#include "GPS.h"


volatile union buffer_element {uint8_t byte;};
volatile union buffer_element gps_fifo_buffer[GPS_RX_BUFFER_SIZE];
volatile fifo_desc_t gps_fifo;

volatile uint16_t packet_index;
volatile uint16_t packet_type = PACKET_NONE;

void parse_str(char*, uint16_t);

double degmin2deg(double, double);
double hrminsec2sec(double hours, double minutes, double seconds);


double utc_time = 0;
double lat = 0;
double lon = 0;
double alt = 0;
double terrainAlt = 0;

uint16_t sats = 0;


ISR(USART_GPS_RXC_vect){
	char trashbin;
	uint8_t data;
	usart_serial_getchar(&USART_GPS, &data);
	
	if(data == '\n' || data == '\r') {
		//idk
	} else if(data == '$') {
		packet_type = PACKET_NONE;
		packet_index = 0;
	} else {
		
		packet_index++;
	}
	
	if(packet_index == 4 && data == 'G') {
		packet_type = PACKET_GGA;
		if(fifo_is_full(&gps_fifo)) {
			fifo_pull_uint8(&gps_fifo, &trashbin);
		}
		fifo_push_uint8(&gps_fifo, '$');
	}
	else if(packet_index == 4) {
		packet_type = PACKET_OTHER;
	}
	
	if(packet_type == PACKET_GGA  && packet_index > 5 && data != '\n' && data != '\r') {
		if(fifo_is_full(&gps_fifo)) {
			fifo_pull_uint8(&gps_fifo, &trashbin);
		}
		fifo_push_uint8(&gps_fifo, data);
	}
}

void gps_init(double zeroAlt) {
	
	//Initialize the GPS
	static usart_serial_options_t gps_usart_options = {
		.baudrate = GPS_USART_SERIAL_BAUDRATE,
		.charlength = GPS_USART_SERIAL_CHAR_LENGTH,
		.paritytype = GPS_USART_SERIAL_PARITY,
		.stopbits = GPS_USART_SERIAL_STOP_BIT
	};
	
	//Sets pins for GPS
	PORTC.DIRSET = 0b10000000;
	PORTC.DIRCLR = 0b01000000;
	PORTC.OUTSET = 0b10000000;
	
	fifo_init(&gps_fifo, gps_fifo_buffer, GPS_RX_BUFFER_SIZE);
	
	usart_serial_init(&USART_GPS, &gps_usart_options);
	usart_set_rx_interrupt_level(&USART_GPS, USART_INT_LVL_HI);
	
	terrainAlt = zeroAlt;
	
}

void gps_update() {
	
//	printf("\n\n");
	char tempChar;
	bool packetStart = false;
	
	char data[64];
	uint16_t commmaCounter = 0;
	uint16_t commaIndex = 0;
	bool readyToParse = false;
	
	while(!fifo_is_empty(&gps_fifo)) {
		fifo_pull_uint8(&gps_fifo, &tempChar);
		/*printf("%c",tempChar);*/
		if(tempChar == '$') {
			packetStart = true;
			commmaCounter = 0;
		} else if(tempChar == '@') {
			packetStart = false;
			commmaCounter = 0;
		}else if(packetStart)	{
			if (tempChar == ',')
			{
				data[commaIndex] = '\0';
				if(commaIndex != 0) {
					parse_str(data, commmaCounter);
				}
				
				commmaCounter++;
				commaIndex = 0;
			}
			else
			{
				data[commaIndex] = tempChar;
				commaIndex++;
			}
			//printf("%c", tempChar);
		}
	}
//	printf("\n\n");
}

//			  1			2		   3 4			 5 6 7  8    9     10 11    12 13 0
// Example: $,030941.00,3443.36038,N,08638.31855,W,1,07,1.04,215.7,M, -30.9,M, ,  *64
// 1: time hhmmss.ss
// 2: latitude ddmm.mmmmm
// 3: N/S
// 4: longitude dddmm.mmmmm
// 5: E/W
// 6: fix indicator
// 7: sats
// 8: hdop
// 9: MSL altitude mmm.m
// 10: M
// 11: Geoid Separation
// 12: M
// 13: N/A
// 14: N/A
// 0: checksum

void parse_str(char* dataString, uint16_t commaCount) {
//	printf("%u: %s\n",commaCount, dataString);
	uint16_t dataStringLength = strlen(dataString);
	if(commaCount == 1) {
		double hours = 0;
		double minutes = 0;
		double seconds = 0;
		for (uint16_t i = 0; i < dataStringLength; i++)
		{
			if(i == 0) {
				hours += (dataString[i]-'0')*10;
			} else if(i == 1) {
				hours += (dataString[i]-'0');
			} else if(i == 2) {
				minutes += (dataString[i]-'0')*10;
			} else if(i == 3) {
				minutes += (dataString[i]-'0');
			} else if(i == 4) {
				seconds += (dataString[i]-'0')*10;
			} else if(i == 5) {
				seconds += (dataString[i]-'0');
			} else if(i == 7) {
				seconds += (dataString[i]-'0')*0.1;
			} else if(i == 8) {
				seconds += (dataString[i]-'0')*0.01;
			}
		}
		utc_time = hrminsec2sec(hours, minutes, seconds);
	}
	else if (commaCount == 2)
	{
		double degrees = 0;
		double minutes = 0;
		for (uint16_t i = 0; i < dataStringLength; i++)
		{
			if(i == 0) {
				degrees += (dataString[i]-'0')*10;
			} else if(i == 1) {
				degrees += (dataString[i]-'0');
			} else if(i == 2) {
				minutes += (dataString[i]-'0')*10;
			} else if(i == 3) {
				minutes += (dataString[i]-'0');
			} else if(i == 5) {
				minutes += (dataString[i]-'0')*0.1;
			} else if(i == 6) {
				minutes += (dataString[i]-'0')*0.01;
			} else if(i == 7) {
				minutes += (dataString[i]-'0')*0.001;
			} else if(i == 8) {
				minutes += (dataString[i]-'0')*0.0001;
			} else if(i == 9) {
				minutes += (dataString[i]-'0')*0.00001;
			}
		}
		
		if(lat < 0) {
			lat = degmin2deg(degrees, minutes) * -1;
		} else {
			lat = degmin2deg(degrees, minutes);
		}
		
	}
	else if (commaCount == 3)
	{
		if ((dataString[0] == 'N' && lat < 0) || (dataString[0] == 'S' && lat > 0))
		{
			lat *= -1;
		}
	}
	else if (commaCount == 4)
	{
		double degrees = 0;
		double minutes = 0;
		for (uint16_t i = 0; i < dataStringLength; i++)
		{
			if(i == 0) {
				degrees += (dataString[i]-'0')*100;
			} else if(i == 1) {
				degrees += (dataString[i]-'0')*10;
			} else if(i == 2) {
				degrees += (dataString[i]-'0');
			} else if(i == 3) {
				minutes += (dataString[i]-'0')*10;
			} else if(i == 4) {
				minutes += (dataString[i]-'0');
			} else if(i == 6) {
				minutes += (dataString[i]-'0')*0.1;
			} else if(i == 7) {
				minutes += (dataString[i]-'0')*0.01;
			} else if(i == 8) {
				minutes += (dataString[i]-'0')*0.001;
			} else if(i == 9) {
				minutes += (dataString[i]-'0')*0.0001;
			} else if(i == 10) {
				minutes += (dataString[i]-'0')*0.00001;
			}
		}
		
		if(lon < 0) {
			lon = degmin2deg(degrees, minutes) * -1;
		} else {
			lon = degmin2deg(degrees, minutes);
		}
	}
	else if (commaCount == 5)
	{
		if ((dataString[0] == 'E' && lon < 0) || (dataString[0] == 'W' && lon > 0))
		{
			lon *= -1;
		}
	}
	else if (commaCount == 7) 
	{
		uint16_t satCount = 0;
		for (uint16_t i = 0; i < dataStringLength; i++)
		{
			if(i == 0) {
				satCount += (dataString[i]-'0')*10;
			} else if(i == 1) {
				satCount += (dataString[i]-'0')*1;
			}
		}
		sats = satCount;
	}
	else if (commaCount == 9)
	{
		double altitude = 0;
		for (uint16_t i = 0; i < dataStringLength; i++)
		{
			if(i == 0) {
				altitude += (dataString[i]-'0')*100;
			} else if(i == 1) {
				altitude += (dataString[i]-'0')*10;
			} else if(i == 2) {
				altitude += (dataString[i]-'0');
			} else if(i == 4) {
				altitude += (dataString[i]-'0')*0.1;
			}
		}
		alt = altitude;
	}
}

double degmin2deg(double degrees, double minutes) {
	return degrees + minutes/60.0;
}

double hrminsec2sec(double hours, double minutes, double seconds) {
	return seconds + minutes*60.0 + hours*60.0*60.0;
}

double gps_get_longitude() {
	return lon;
}

double gps_get_latitude() {
	return lat;
}

double gps_get_altitude() {
	return alt;
}

double gps_get_time() {
	return utc_time;
}

uint16_t gps_get_sats() {
	return sats;
}