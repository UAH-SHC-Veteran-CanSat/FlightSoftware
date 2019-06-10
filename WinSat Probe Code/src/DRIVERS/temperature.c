/*
 * temperature.c
 *
 * Created: 4/3/2019 2:57:36 PM
 *  Author: Mason
 */ 
#include <asf.h>
#include <math.h>
#include "temperature.h"

/*
void adc_init(void){	//This is for PA5
	PORTA.DIRCLR = 0b00100000;
	PORTA.DIR = 0b11011111;
	ADCA.CTRLA = 0b00000001;
	ADCA.CTRLB = 0b00000000;
	ADCA.REFCTRL = 0b00010000;
	ADCA.PRESCALER = 0b00000101;
	ADCA.CAL = adc_get_calibration_data(ADC_CAL_ADCA);
	
	ADCA.CH0.CTRL = 0b00000001;
	ADCA.CH0.MUXCTRL = 0b00000101;
}*/



#define MY_ADC ADCA
#define MY_ADC_CH ADC_CH0

void adc_init(void) {
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1);
	
	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}
/*
float getVoltage(void){
	ADCA.CH0.CTRL  |= 0b10000000;	
	("one");
	while(ADCA.CH0.INTFLAGS == 0);
	//printf("two");
	ADCA.CH0.INTFLAGS = 0;
	//printf("three");
	uint16_t adcReading = ADCA.CH0.RES;
	printf("ADC reading = %u\n", adcReading);
	float voltage = 0.0004899 * (float)(adcReading) - 0.0856326;	//We have to find these numbers by applying differing voltage, printing ADC readings, and solve equation
	printf("voltage: %f \n", voltage);
	return voltage;
	
}
*/

float getVoltage(void){
	uint16_t adcReading;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
	adcReading = adc_get_result(&MY_ADC, MY_ADC_CH);
	adc_disable(&MY_ADC);
	//printf("ADC reading = %u\n", adcReading);
	//float voltage = 0.0009594 * (float)(adcReading);	//We have to find these numbers by applying differing voltage, printing ADC readings, and solve equation
	float voltage = -0.0005 * (float)(adcReading) + 3.3699;
	//printf("voltage: %f \n", voltage);
	return voltage;
}
/*
uint16_t getCurrent(void){
	uint16_t voltage = getVoltage();
	uint16_t current = (voltage/10000);
	return current;
}
*/
float getTemperature(void){
	float A, B, C, D, A1, B1, C1, D1, rRef;
	//A = -10.2296, B = 2887.62, C = 132336, D = -25025100, A1 = 3.354016*pow(10, -3), B1 = 3.415560*pow(10, -4), C1 = 4.95445*pow(10, -6), D1 = 4.3642236*pow(10, -7), rRef = 20000;
	A = -14.6337, B = 4791.842, C = -115334, D = -3.730535*pow(10, 6), A1 = 3.354016*pow(10, -3), B1 = 2.569850*pow(10, -4), C1 = 2.620131*pow(10, -6), D1 = 6.3830991*pow(10, -8), rRef = 10000;
	float voltage = getVoltage();
	//uint16_t current = getCurrent();
	//uint16_t resistance = voltage/current;
	//float resistance = (voltage*10000)/(voltage-3.3);
	//float resistance = 20000 * (3.3/voltage-1);
	//float resistance = rRef*(exp(A + (B/298) + (C/pow(298, 2)) + (D/pow(298, 3))));
	//float resistance = (3.3*4700 - voltage*4700)/voltage;
	float resistance = ((voltage/3.3)*4700)/(1-(voltage/3.3));
	//printf("resistance = %f \n", resistance);
	//float temperature = 3977.0/(log(resistance/(10000.0*pow(2.71828,(-3977.0/298.15)))));
	float temperature = pow(A1 + B1*log(resistance/rRef) + C1*pow(log(resistance/rRef), 2) + D1*pow(log(resistance/rRef), 3), -1);
	temperature = temperature - 273.15;
	//float temperature = pow((.003351016+.0002569850*log(resistance/10000)+.000002620131*pow(log(resistance/10000),2)),-1);
	return temperature;	
}