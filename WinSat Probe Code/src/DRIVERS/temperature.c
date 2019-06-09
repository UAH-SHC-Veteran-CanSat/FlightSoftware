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
	printf("ADC reading = %u\n", adcReading);
	float voltage = 0.0004899 * (float)(adcReading) - 0.0856326;	//We have to find these numbers by applying differing voltage, printing ADC readings, and solve equation
	printf("voltage: %f \n", voltage);
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
	float voltage = getVoltage();
	//uint16_t current = getCurrent();
	//uint16_t resistance = voltage/current;
	//float resistance = (voltage*10000)/(voltage-3.3);
	float resistance = 10000 * (3.3/voltage-1);
	//printf("resistance = %f \n", resistance);
	uint32_t temperature = 3977.0/(log(resistance/(10000.0*pow(2.71828,(-3977.0/298.15)))));
	//float temperature = pow((.003351016+.0002569850*log(resistance/10000)+.000002620131*pow(log(resistance/10000),2)),-1);
	return temperature;	
}