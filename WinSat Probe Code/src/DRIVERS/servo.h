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

void servos_stop();
void servos_start();

void fin1_init(uint16_t minPos, uint16_t maxPos);

void fin1_disable();
void fin1_set_duty(uint16_t duty);
void fin1_set_pos(uint16_t permille);

void fin2_init(uint16_t minPos, uint16_t maxPos);

void fin2_disable();
void fin2_set_duty(uint16_t duty);
void fin2_set_pos(uint16_t permille);

void release_init(uint16_t minPos, uint16_t maxPos);

void release_open(); //this drops the cansat
void release_close(); //this grabs the cansat
void release_limp(); //this takes power away from the release servo
void release_set_duty(uint16_t duty);



#endif /* SERVO_H_ */