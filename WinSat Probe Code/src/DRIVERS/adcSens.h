/*
 * temperature.h
 *
 * Created: 4/3/2019 2:58:03 PM
 *  Author: Mason
 */ 


#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <asf.h>

#define MY_ADC ADCA
#define THERM_ADC_CH ADC_CH0
#define PWR_ADC_CH ADC_CH1

void adc_init(void);
float adc_get_therm_voltage(void);
float adc_get_pwr_voltage(void);
//uint16_t getCurrent(void);
float adc_get_temperature(void);



#endif /* TEMPERATURE_H_ */