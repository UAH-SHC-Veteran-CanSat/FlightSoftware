/*
 * dataSaver.c
 *
 * Created: 6/13/2019 6:46:47 PM
 *  Author: trbinsc
 */ 

#include "dataSaver.h"

void save_state(uint16_t state) // 0 to 1
{
	ee_write_uint16(state, 0);
}

void save_packets(uint32_t packets) // 2 to 5
{
	ee_write_uint32(packets, 2);
}

void save_time(uint32_t secs) //6 to 9
{
	ee_write_uint32(secs, 6);
}

void save_utc(uint32_t utcSecs) //10 to 13
{
	ee_write_uint32(utcSecs, 10);
}

void save_ground_alt(double zeroAlt) // 14 to 17
{
	ee_write_double(zeroAlt, 14);
}


uint16_t read_state(void) // 0 to 1
{
	return ee_read_uint16(0);
}

uint32_t read_packets(void) // 2 to 5
{
	return ee_read_uint32(2);
}

uint32_t read_time(void) //6 to 9
{
	return ee_read_uint32(6);
}

uint32_t read_utc(void) //10 to 13
{
	return ee_read_uint32(10);
}

double read_ground_alt(void) // 14 to 17
{
	return ee_read_double(14);
}
