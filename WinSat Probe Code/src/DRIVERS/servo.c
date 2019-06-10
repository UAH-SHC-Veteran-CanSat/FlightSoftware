/*
 * servo.c
 *
 * Created: 6/9/2019 3:25:06 PM
 *  Author: Mason
 */ 

#include "servo.h"

uint16_t fin1_min = 0;
uint16_t fin1_max = 10000;

uint16_t fin2_min = 0;
uint16_t fin2_max = 10000;

uint16_t release_min = 0;
uint16_t release_max = 10000;

void servos_stop()
{
	tc_write_clock_source(&TCE0, TC_CLKSEL_OFF_gc);
}

void servos_start()
{
	tc_write_clock_source(&TCE0, TC_CLKSEL_DIV64_gc);
}

void fin1_init(uint16_t minPos, uint16_t maxPos) 
{
	fin1_min = minPos;
	fin1_max = maxPos;
	
	PORTE.DIRSET = 0b00001110;
	
	sysclk_enable_peripheral_clock(&TCE0);
	
	tc_enable(&TCE0);
	tc_set_wgm(&TCE0, TC_WG_SS);
	tc_write_period(&TCE0, 10000);
	tc_write_cc(&TCE0, TC_CCB, (fin1_max+fin1_min)/2); 
	tc_enable_cc_channels(&TCE0,TC_CCBEN); 
}

void fin1_disable()
{
	fin1_set_duty(0);
}

void fin1_set_duty(uint16_t duty)
{
	tc_write_cc(&TCE0, TC_CCB, duty);
}

void fin1_set_pos(uint16_t permille)
{
	uint16_t duty = (uint16_t)( fin1_min + ((uint32_t)(fin1_max-fin1_min)*permille)/1000);
	fin1_set_duty(duty);
}

void fin2_init(uint16_t minPos, uint16_t maxPos)
{
	fin2_min = minPos;
	fin2_max = maxPos;
	
	PORTE.DIRSET = 0b00001110;
	
	sysclk_enable_peripheral_clock(&TCE0);
	
	tc_enable(&TCE0);
	tc_set_wgm(&TCE0, TC_WG_SS);
	tc_write_period(&TCE0, 10000);
	tc_write_cc(&TCE0, TC_CCC, (fin2_max+fin2_min)/2);
	tc_enable_cc_channels(&TCE0,TC_CCCEN);
}

void fin2_disable()
{
	fin2_set_duty(0);
}

void fin2_set_duty(uint16_t duty)
{
	tc_write_cc(&TCE0, TC_CCC, duty);
}

void fin2_set_pos(uint16_t permille)
{
	uint16_t duty = (uint16_t)( fin2_min + ((uint32_t)(fin2_max-fin2_min)*permille)/1000);
	fin2_set_duty(duty);
}


void release_init(uint16_t minPos, uint16_t maxPos)
{
	release_min = minPos;
	release_max = maxPos;
	
	PORTE.DIRSET = 0b00001110;
	
	sysclk_enable_peripheral_clock(&TCE0);
	
	tc_enable(&TCE0);
	tc_set_wgm(&TCE0, TC_WG_SS);
	tc_write_period(&TCE0, 10000);
	tc_write_cc(&TCE0, TC_CCD, 0);
	tc_enable_cc_channels(&TCE0,TC_CCDEN);
}

void release_open()
{
	release_set_duty(release_min);
}

void release_close()
{
	release_set_duty(release_max);
}
void release_limp()
{
	release_set_duty(0);
}

void release_set_duty(uint16_t duty)
{
	tc_write_cc(&TCE0, TC_CCD, duty);
}




