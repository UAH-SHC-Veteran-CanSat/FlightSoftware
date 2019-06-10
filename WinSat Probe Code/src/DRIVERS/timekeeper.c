/*
 * timekeeper.c
 *
 * Created: 6/9/2019 8:41:59 PM
 *  Author: trbinsc
 */ 

#include "timekeeper.h"

static void timekeeper_callback(void);

uint32_t last_utc_time = 0;
uint32_t last_utc_millis = 0;

uint32_t millis = 0;

uint16_t period = 33240;

uint32_t loop_start = 0;

void timekeeper_init()
{
	tc_enable(&TK_TC);
	tc_set_overflow_interrupt_callback(&TK_TC, timekeeper_callback);
	tc_set_wgm(&TK_TC, TC_WG_NORMAL);
	tc_write_period(&TK_TC, period);
	tc_set_overflow_interrupt_level(&TK_TC, TC_INT_LVL_LO);
	tc_write_clock_source(&TK_TC, TC_CLKSEL_DIV1_gc);
}

void timekeeper_refine(uint32_t utc_time)
{
	if(last_utc_time == 0)
	{
		//printf("Last time is zero\n");	
		last_utc_time = utc_time;
		last_utc_millis = millis;
	}
	else if(last_utc_time != utc_time)
	{
		printf("Actual time: %lu, Guessed time: %lu, ", (utc_time-last_utc_time)*1000, (millis-last_utc_millis));	
		period = (uint32_t)((1.0-TK_ADJUST_SPEED)*period + TK_ADJUST_SPEED*period * ((double)(millis-last_utc_millis))/((double)((utc_time-last_utc_time)*1000)));
		printf("New period: %u\n",period);
		tc_write_period(&TK_TC, period);
		last_utc_time = utc_time;
		last_utc_millis = millis;
	}
	else
	{
		//printf("Last time is equal to current time\n");	
	}
}

static void timekeeper_callback(void)
{
	millis++;
	tc_clear_overflow(&TK_TC);
}

uint32_t timekeeper_get_millis()
{
	return millis;
}

uint32_t timekeeper_get_sec()
{
	return millis/1000;
}

void timekeeper_delay_ms(uint32_t delay_time)
{
	timekeeper_delay_until_ms(delay_time+millis);
}

void timekeeper_delay_until_ms(uint32_t delay_time)
{
	while(millis < delay_time)
	{
		delay_ms(1);
	}
}

void timekeeper_loop_start()
{
	loop_start = millis;
}

void timekeeper_loop_end(uint32_t loop_period)
{
	if(millis < loop_start + loop_period)
	{
		timekeeper_delay_until_ms(loop_period+loop_start);
	}
	else
	{
		printf("Period too low, can't keep up");
	}
	
}