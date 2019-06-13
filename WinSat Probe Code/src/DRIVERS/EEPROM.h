/*
 * EEPROM.h
 *
 * Created: 6/13/2019 7:07:08 AM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>

#ifndef EEPROM_H_
#define EEPROM_H_

void ee_init(void);

void ee_write_uint16(uint16_t data, uint32_t address);
void ee_write_uint32(uint32_t data, uint32_t address);
void ee_write_int16(int16_t data, uint32_t address);
void ee_write_int32(int32_t data, uint32_t address);
void ee_write_double(double data, uint32_t address);


uint16_t ee_read_uint16(uint32_t address);
uint32_t ee_read_uint32(uint32_t address);
int16_t ee_read_int16(uint32_t address);
int32_t ee_read_int32(uint32_t address);
double ee_read_double(uint32_t address);




#endif /* EEPROM_H_ */