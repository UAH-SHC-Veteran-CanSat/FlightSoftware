/*
 * usart_command.h
 *
 * Created: 5/30/2017 5:50:23 PM
 *  Author: trb0023
 */ 

#include <asf.h>

#ifndef USART_COMMAND_H_
#define USART_COMMAND_H_

uint8_t is_command_ready(void);
char get_command(void);

#endif /* USART_COMMAND_H_ */