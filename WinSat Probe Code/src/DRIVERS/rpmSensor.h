/*
 * rpmSensor.h
 *
 * Created: 6/10/2019 3:58:05 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>
#include "timekeeper.h"

#ifndef RPMSENSOR_H_
#define RPMSENSOR_H_

//23/64 of 3.3v
#define VOLTAGE_SCALED_VALUE 21

//Any rotation with a period smaller than this value will be assumed to be zero
#define MIN_ROTATION_PERIOD 5000

void rpm_init();

uint32_t rpm_get_rate();



#endif /* RPMSENSOR_H_ */