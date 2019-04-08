/*
 * altitude.c
 *
 * Created: 4/2/2019 12:29:06 PM
 *  Author: Mason
 */ 

int32_t getAltitude(uint32_t initial, uint32_t pressure, int32_t temperature){
	//printf("pressure = %lu \n", pressure);
	//printf("temperature = %i \n", (uint16_t) TEMP);
	int32_t altitude = (int32_t)(((287.058 * (((float)(TEMP)/100)+273.15)/9.8))*log((float)initial/get_pressure())*3.28);
	//printf("altitude = %i \n", (int16_t)altitude);
	return altitude;