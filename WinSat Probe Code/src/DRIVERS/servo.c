/*
 * servo.c
 *
 * Created: 6/9/2019 3:25:06 PM
 *  Author: Mason
 */ 

#include "servo.h"

struct pwm_config fin1_cfg;
struct pwm_config fin2_cfg;
struct pwm_config release_cfg;

void fin1_init(void) {
	
	pwm_init(&fin1_cfg, PWM_TCE0, PWM_CH_B, 250); //Supposedly 250 hz but I don't think it really is
}

void fin1_stop()
{
	pwm_stop(&fin1_cfg);
}

void fin1_start()
{
	pwm_start(&fin1_cfg, 50);
}

void fin1_set_duty(uint8_t duty)
{
	pwm_set_duty_cycle_percent(&fin1_cfg, duty);
}

void fin2_init(void) {
	
	pwm_init(&fin2_cfg, PWM_TCE0, PWM_CH_C, 250); //Supposedly 250 hz but I don't think it really is
}

void fin2_stop()
{
	pwm_stop(&fin2_cfg);
}

void fin2_start()
{
	pwm_start(&fin2_cfg, 50);
}

void fin2_set_duty(uint8_t duty)
{
	pwm_set_duty_cycle_percent(&fin2_cfg, duty);
}

void fin12_init(void) 
{
	fin1_init();
	fin2_init();
}

void fin12_stop()
{
	fin1_stop();
	fin2_stop();
}

void fin12_start()
{
	fin1_start();
	fin2_start();
}

void fin12_set_duty(uint8_t duty)
{
	fin1_set_duty(duty);
	fin2_set_duty(duty);
}




