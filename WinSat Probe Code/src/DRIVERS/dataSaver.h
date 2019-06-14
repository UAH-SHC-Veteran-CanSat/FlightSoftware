/*
 * dataSaver.h
 *
 * Created: 6/13/2019 6:46:59 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include "DRIVERS/eeprom.h"


#ifndef DATASAVER_H_
#define DATASAVER_H_


void save_state(uint16_t state); // 0 to 1
void save_packets(uint32_t packets); // 2 to 5
void save_time(uint32_t secs); //6 to 9
void save_utc(uint32_t utcSecs); //10 to 13
void save_ground_alt(double zeroAlt); // 14 to 17

uint16_t read_state(void); // 0 to 1
uint32_t read_packets(void); // 2 to 5
uint32_t read_time(void); //6 to 9
uint32_t read_utc(void); //10 to 13
double read_ground_alt(void); // 14 to 17


#endif /* DATASAVER_H_ */