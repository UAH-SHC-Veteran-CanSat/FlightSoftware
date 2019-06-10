/*
 * altimeter.h
 *
 * Created: 6/8/2019 7:38:18 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>
#include <math.h>
#include "DRIVERS/bmp280.h"

#ifndef ALTIMETER_H_
#define ALTIMETER_H_

#define	I2C_BUFFER_LEN 64
#define ALT_TWI TWIC

#define PRES_HIST_BUFFER_LEN 8

void alt_init();
void alt_set_zero(double);
void alt_set_current_to_zero();

void alt_update();

double alt_get_pressure();
double alt_get_temperature();
double alt_get_current_altitude();
double alt_get_smooth_altitude();
double alt_get_current_vvel(double);
double alt_get_smooth_vvel(double);



#endif /* ALTIMETER_H_ */