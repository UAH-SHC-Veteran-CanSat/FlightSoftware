/*
 * rpmSensor.h
 *
 * Created: 6/10/2019 3:58:05 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>

#ifndef RPMSENSOR_H_
#define RPMSENSOR_H_

//23/64 of 3.3v
#define VOLTAGE_SCALED_VALUE 21

void rpm_init();

void rpm_clear_counts();
uint32_t rpm_peek_counts();
uint32_t rpm_pull_counts();

uint32_t rpm_get_rate(uint32_t call_millis);



#endif /* RPMSENSOR_H_ */