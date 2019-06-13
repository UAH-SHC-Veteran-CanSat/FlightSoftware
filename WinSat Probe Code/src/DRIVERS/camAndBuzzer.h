/*
 * camAndBuzzer.h
 *
 * Created: 6/12/2019 5:54:51 PM
 *  Author: trbinsc
 */ 

#include <asf.h>

#ifndef CAMANDBUZZER_H_
#define CAMANDBUZZER_H_

void cambuz_init();

void cam_enable();
void cam_disable();

void buz_enable();
void buz_disable();

void cam_update(uint32_t millis);
void buz_update(uint32_t millis);



#endif /* CAMANDBUZZER_H_ */