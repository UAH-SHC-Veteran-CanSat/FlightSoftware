/*
 * camAndBuzzer.c
 *
 * Created: 6/12/2019 5:54:39 PM
 *  Author: trbinsc
 */ 

#include "camAndBuzzer.h"

uint32_t buzSeq[44] = {100, 1000, 100, 900, 100, 800, 100, 700, 100, 600, 100, 500, 100, 400, 100, 300, 100, 200, 100, 100, 100, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 2000};
uint32_t buzSeqLen = 44;
uint32_t buzLastTime = 0;
uint32_t buzIndex = 0;
bool buzEnabled = false;


uint32_t camOnMillis = 0;
bool lastCamOn = false;
bool camOn = false;

void cambuz_init()
{
	PORTC.DIRSET = 0b00010000;
	PORTD.DIRSET = 0b00010000;
}

void cam_enable()
{
	camOn = true;
}

void cam_disable()
{
	camOn = false;
}

void buz_enable()
{
	buzEnabled = true;
}

void buz_disable()
{
	buzEnabled = false;
}

void cam_update(uint32_t millis)
{
	if(camOn != lastCamOn)
	{
		lastCamOn = camOn;
		
		camOnMillis = millis + 750;
		PORTC.OUTSET = 0b00010000;
	}
	
	if(millis > camOnMillis)
	{
		PORTC.OUTCLR = 0b00010000;
	}
	
}

void buz_update(uint32_t millis)
{
	if(millis > buzSeq[buzIndex] + buzLastTime)
	{
		buzLastTime = millis;
		if(buzEnabled)
		{
			buzIndex = (buzIndex+1)%buzSeqLen;
			if(buzIndex%2 == 0)
			{
				PORTD.OUTSET = 0b00010000;
			}
			else
			{
				PORTD.OUTCLR = 0b00010000;
			}
		}
		else
		{
			buzIndex = 0;
			PORTD.OUTCLR = 0b00010000;
		}
	}
}