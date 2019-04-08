/*
 * temperature.h
 *
 * Created: 4/3/2019 2:58:03 PM
 *  Author: Mason
 */ 


#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <asf.h>

void adc_init(void);
float getVoltage(void);
//uint16_t getCurrent(void);
float getTemperature(void);



#endif /* TEMPERATURE_H_ */