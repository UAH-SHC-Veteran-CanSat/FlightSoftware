/*
 * pid.c
 *
 * Created: 6/10/2019 8:52:33 PM
 *  Author: trbinsc
 */ 

#include "pid.h"

double kp, ki, kd;
double input, output, setpoint;
double errSum, lastErr;

uint32_t last_millis = 0;

void pid_init(double setkp, double setki, double setkd, double setsetpoint)
{
	pid_set_gains(setkp, setki, setkd);
	pid_set_setpoint(setsetpoint);
}

void pid_set_gains(double setkp, double setki, double setkd)
{
	kp = setkp;
	ki = setki;
	kd = setkd;
}

void pid_set_setpoint(double setsetpoint)
{
	setpoint = setsetpoint;
}

void pid_update (double heading, uint32_t millis)
{
	double dt = ((double)(millis - last_millis))/1000.0;
	
	double error = setpoint - heading; //TODO: redo this to work for a 360 degree rotation
	
	errSum += (error * dt);
	double dErr = (error - lastErr) / dt;
	
	output = kp*error + ki*errSum + kd*dErr;
	
	last_millis = millis;
	lastErr = error;
}

uint16_t pid_output()
{
	return -output*2 + 500; // add 500 so it is centered
}