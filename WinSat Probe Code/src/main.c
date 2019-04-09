/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "altitude.h"
#include "pressure.h"
#include "temperature.h"

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	//sysclk_enable_peripheral_clock(&TCF0);
	sysclk_enable_peripheral_clock(&TCD0);


	sysclk_enable_module(SYSCLK_PORT_C, PR_TWI_bm);
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_E, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_F, SYSCLK_HIRES);
	
	sysclk_enable_peripheral_clock(&TCE0);
	sysclk_enable_peripheral_clock(&TCC0);

	sysclk_enable_peripheral_clock(&USARTC0);
	sysclk_enable_peripheral_clock(&USARTD0);
	sysclk_enable_peripheral_clock(&USARTD1);
	
	sysclk_enable_peripheral_clock(&TWIC);
	
	sysclk_enable_peripheral_clock(&ADCA);
	adc_init();
	
	/* Insert application code here, after the board has been initialized. */
	uint8_t state = 0;
	uint8_t teamID = 2591;
	uint32_t pressure = getPressure();
	int32_t temperature = getTemperature();
	uint32_t initialPressure = getPressure();
	int32_t initialAltitude = getAltitude(initialPressure,pressure);
	int32_t maxAltitude = 0;
	int32_t altitude = 0;
	int32_t velocity = 0;
	int32_t smooth_altitude = 0;
	double smoothing_factor = 0.50;
	
	
	while (1){
		pressure = getPressure();
		altitude = getAltitude(initialPressure, pressure, temperature);
		//velocity = getVelocity();
		smooth_altitude = (int32_t)(smoothing_factor * altitude + (1-smoothing_factor)*smooth_altitude);
		
		//teamID,my_time,packetCount,altitude,pressure,temperature,voltage,GPSTime,GPSLat,GPSLong,GPSAlt,GPSSats,smooth_x,smooth_y,smooth_z,state
		
		if (state == 0){
			printf("Flight State 0");
			if ((altitude <= 500) && ((int32_t)maxAltitude - (int32_t)smooth_altitude <= 0)){ //Work on Velocity later, this will work for now
				state = 1;
			}
			if (smooth_altitude > maxAltitude) {
				maxAltitude = smooth_altitude;
			}
		}
		if (state == 1){
			printf("Flight State 1");
			if (smooth_altitude-initialAltitude<=450){
				state = 2;
			}
		}
		if (state == 2){
			printf("Flight State 2");
			if ((smooth_altitude - initialAltitude <= 50) && (velocity<=1)){
				state = 3;
			}
		}
	}
}
