/*
 * timekeeper.h
 *
 * Created: 6/9/2019 8:42:25 PM
 *  Author: trbinsc
 */ 


#ifndef TIMEKEEPER_H_
#define TIMEKEEPER_H_

#include <asf.h>

#define TK_TC TCD0
#define TK_ADJUST_SPEED 0.01

void timekeeper_init();

void timekeeper_refine(uint32_t utc_time);

uint32_t timekeeper_get_millis();

uint32_t timekeeper_get_sec();

void timekeeper_delay_ms(uint32_t delay_time);
void timekeeper_delay_until_ms(uint32_t delay_time);

void timekeeper_loop_start();
void timekeeper_loop_end(uint32_t loop_period);


#endif /* TIMEKEEPER_H_ */