/*
 * rpmSensor.c
 *
 * Created: 6/10/2019 3:57:54 PM
 *  Author: trbinsc
 */ 

#include "rpmSensor.h"

static void rpm_callback(AC_t *ac, uint8_t channel, enum ac_status_t status);

volatile uint32_t last_rate_millis = 0;
volatile uint32_t current_rate_millis = 0;

static void rpm_callback(AC_t *ac, uint8_t channel, enum ac_status_t status)
{
	last_rate_millis = current_rate_millis;
	current_rate_millis = timekeeper_get_millis();
}

void rpm_init()
{
	struct ac_config aca_config;
	
	memset(&aca_config, 0, sizeof(struct ac_config));
	
	ac_set_mode(&aca_config, AC_MODE_SINGLE);
	ac_set_voltage_scaler(&aca_config, VOLTAGE_SCALED_VALUE-1);
	ac_set_hysteresis(&aca_config, AC_HYSMODE_LARGE_gc);
	ac_set_negative_reference(&aca_config, AC_MUXNEG_SCALER_gc);
	ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN1_gc);
	
	ac_set_interrupt_mode(&aca_config, AC_INT_MODE_RISING_EDGE);
	ac_set_interrupt_level(&aca_config, AC_INT_LVL_MED);
	ac_set_interrupt_callback(&ACA, rpm_callback);
	
	ac_write_config(&ACA, 1, &aca_config);
	
	ac_enable(&ACA, 1);
	
}

uint32_t rpm_get_rate()
{
	if(timekeeper_get_millis() < current_rate_millis + 5000 && current_rate_millis != last_rate_millis)
	{
		return 60000/(current_rate_millis-last_rate_millis);
	}
	else
	{
		return 0;
	}
	
	
}