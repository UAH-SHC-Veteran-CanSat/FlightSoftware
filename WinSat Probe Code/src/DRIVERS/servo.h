/*
 * servo.h
 *
 * Created: 6/9/2019 3:25:20 PM
 *  Author: Mason
 */ 

#include <asf.h>

#ifndef SERVO_H_
#define SERVO_H_

/*
 * servo.c
 *
 * Created: 6/9/2019 3:25:06 PM
 *  Author: Mason
 */ 

#include "servo.h"


void fin1_init(void);

void fin1_stop();

void fin1_start();

void fin1_set_duty(uint8_t duty);

void fin2_init(void);

void fin2_stop();

void fin2_start();

void fin2_set_duty(uint8_t duty);

void fin12_init(void);

void fin12_stop();

void fin12_start();

void fin12_set_duty(uint8_t duty);







#endif /* SERVO_H_ */