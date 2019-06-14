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

#define PRES_HIST_BUFFER_LEN 16

void alt_init(void);
double alt_get_zero(void);
void alt_set_zero(double);
void alt_set_current_to_zero(void);

void alt_update(void);

double alt_get_pressure(void);
double alt_get_temperature(void);
double alt_get_current_altitude(void);
double alt_get_smooth_altitude(void);
double alt_get_current_vvel(double);
double alt_get_smooth_vvel(double);



#endif /* ALTIMETER_H_ */