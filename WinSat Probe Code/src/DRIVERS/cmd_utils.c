/*
 * cmd_utils.c
 *
 * Created: 6/11/2019 3:34:21 PM
 *  Author: trbinsc
 */ 

#include "cmd_utils.h"

double cmd_parse_double(char* string)
{
	return atof(string);
}

uint32_t cmd_parse_uint32(char* string)
{
	return strtoul(string, NULL, 10);
}

uint16_t cmd_parse_uint16(char* string)
{
	return (uint16_t) strtoul(string, NULL, 10);
}

int32_t cmd_parse_int32(char* string)
{
	return strtol(string, NULL, 10);
}

int16_t cmd_parse_int16(char* string)
{
	return (int16_t) strtol(string, NULL, 10);
}

char* cmd_split(char* string, char* splitchar, uint16_t index)
{
	char stringCopy[256];
	strcpy(stringCopy,string);
	
	char *ptr = strtok(stringCopy, splitchar);
	
	for (uint16_t i = 0; i < index; i++)
	{
		ptr = strtok(NULL, splitchar);
	}
	
	return ptr;
}

void cmd_parse_pid_values(char* string, double *kp, double *ki, double *kd)
{
	*kp = atof(cmd_split(string,"/",1));
	*ki = atof(cmd_split(string,"/",2));
	*kd = atof(cmd_split(string,"/",3));
}