/*
 * cmd_utils.h
 *
 * Created: 6/11/2019 3:34:33 PM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>

#ifndef CMD_UTILS_H_
#define CMD_UTILS_H_



double cmd_parse_double(char* string);

uint32_t cmd_parse_uint32(char* string);
uint16_t cmd_parse_uint16(char* string);
int32_t cmd_parse_int32(char* string);
int16_t cmd_parse_int16(char* string);

char* cmd_split(char* string, char* splitchar, uint16_t index);

void cmd_parse_pid_values(char* string, double *kp, double *ki, double *kd);


#endif /* CMD_UTILS_H_ */