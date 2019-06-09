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
#include <math.h>
#include <string.h>
#include "DRIVERS/usart.h"
#include "DRIVERS/GPS.h"
#include "DRIVERS/altimeter.h"
//#include "DRIVERS/pressure.h"
#include "DRIVERS/temperature.h"
#include "DRIVERS/IMU.h"



//Thomas' Code---------------------------------------------------------------------------------------------------

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	
	pmic_init();

	sysclk_enable_module(SYSCLK_PORT_C, PR_TWI_bm);
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_E, SYSCLK_HIRES);
	sysclk_enable_module(SYSCLK_PORT_F, SYSCLK_HIRES);
	
	sysclk_enable_peripheral_clock(&TCD0);
	sysclk_enable_peripheral_clock(&TCE0);
	sysclk_enable_peripheral_clock(&TCC0);

	sysclk_enable_peripheral_clock(&USARTC0);
	sysclk_enable_peripheral_clock(&USARTC1);
	sysclk_enable_peripheral_clock(&USARTD1);
	
	UART_Comms_Init();
	
	printf("\n\n\nUSART INIT\n"); //This prints to xbee
	log_printf("\n\n\nSECONDARY COMMS INIT\n"); //This prints to openlog
	log_printf("Test: %u, %u",69, 420);
	
	gps_init(0.0);
	
	
	irq_initialize_vectors();
	cpu_irq_enable();
	
	
	sysclk_enable_peripheral_clock(&TWIC);
	
	sysclk_enable_peripheral_clock(&ADCA);
	adc_init();
	
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_1KCLK);
	//wdt_enable();
	

	

	TCE0.CTRLA = 0b00000110;
	TCE0.PER = 121;
	TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	wdt_reset();

	imu_init();

	wdt_reset();

	alt_init();

	printf("\nInitialization Complete!\n\n\n");
	printf("Testing float print %f\n",420.69);
	
	delay_ms(500);
	
	

// 	/* Insert application code here, after the board has been initialized. */
	uint8_t state = 0;
	uint16_t teamID = 2591;
	int8_t initialPressure = 0;//bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	int8_t pressure =  0;//bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	int32_t temperature = getTemperature();
	//int32_t initialAltitude = getAltitude(initialPressure,pressure,temperature);
	int32_t maxAltitude = 0;
	int32_t altitude = 0;
	int32_t oldAltitude = 0;
	int32_t newAltitude = 0;
	//int32_t velocity = 0;
	int32_t smooth_altitude = 0;
	int32_t smoothOldAltitude = 0;
	int32_t smoothNewAltitude = 0;
	double smoothing_factor = 0.50;
	int32_t altitudeArray[10];
	int32_t altitudeChange = 0;
	double newTime = 0;
	double oldTime = 0;	
	
	while (1){
		
//		printf("hi\n");
		if(is_command_ready()){
			printf("%s\n",get_command());
		}
		gps_update();
// 		printf("time:%f\n",gps_get_time());
// 		printf("lat :%f\n",gps_get_latitude());
// 		printf("lon :%f\n",gps_get_longitude());
// 		printf("alt :%f\n",gps_get_altitude());
// 		printf("sats:%u\n\n",gps_get_sats());

		printf("\n\n\n\n");
		delay_ms(1000);
		newTime = gps_get_time();
		printf("New Altitude: %li, New Time: %f\n", smoothNewAltitude, newTime);
		//printf("temperature: %f\n\n",getTemperature());

		
/*		printf("temperature: %f\n\n",getTemperature());*/
		
		imu_update();
		
		printf("2591,0,0,0,0,0,0,0,0,0,0,0,%.0f,%.0f,00,PRELAUNCH,%.0f\n",imu_pitch(), imu_roll(), imu_heading());
		//printf("CALBRATION STATUSES:  Accel: %u, Gyro: %u, Mag: %u, Sys: %u\n", imu_accel_cal(), imu_gyro_cal(), imu_mag_cal(), imu_sys_cal());
		
		alt_update();
		
		printf("temp: %f, pres: %f\n",alt_get_temperature(), alt_get_pressure());

		if (state == 0){
			printf("Flight State 0\n");
			if ((smooth_altitude <= 500) && ((int32_t)maxAltitude - (int32_t)smooth_altitude <= -10)){ //Work on Velocity later, this will work for now
			//if ((smooth_altitude <= 500) && (altitudeChange <= -10)){
				state = 1;
			}
			if (smooth_altitude > maxAltitude) {
				maxAltitude = smooth_altitude;
 			}
		}
		if (state == 1){
			printf("Flight State 1\n");
// 			if (smooth_altitude-initialAltitude<=450){
// 				state = 2;
// 			}
		}
		if (state == 2){
			printf("Flight State 2\n");
// 			if ((smooth_altitude - initialAltitude <= 50) /*&& (velocity<=1)*/){
// 				state = 3;
// 			}
		}
		if (state == 3){
			printf("Flight State 3\n");
			//Buzzer or something
		}
//		oldAltitude = getAltitude(initialPressure,pressure,temperature);
//		smoothOldAltitude = (int32_t)(smoothing_factor * oldAltitude + (1-smoothing_factor)*smoothOldAltitude);
		oldTime = gps_get_time();
//		printf("Old Altitude: %li, Old Time: %f\n", smoothOldAltitude, oldTime);
 	}
}
