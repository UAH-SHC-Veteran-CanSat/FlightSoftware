/*
 * Cansat 2019 Astrotrain Flight Software
 * Programmed with love by Mason Barrow and Thomas Bennett
 * <3
 */
#include <asf.h>
#include <math.h>
#include <string.h>
#include "DRIVERS/timekeeper.h"
#include "DRIVERS/usart.h"
#include "DRIVERS/GPS.h"
#include "DRIVERS/altimeter.h"
#include "DRIVERS/adcSens.h"
#include "DRIVERS/IMU.h"
#include "DRIVERS/servo.h"
#include "DRIVERS/rpmSensor.h"
#include "DRIVERS/pid.h"
#include "DRIVERS/cmd_utils.h"

uint16_t state = 0;
uint32_t telemetryPeriod = 1000;
uint32_t logPeriod = 200;
uint32_t gpsPeriod = 1000;
uint32_t lastPacketMillis = 0;
uint32_t lastLogMillis = 0;
uint32_t lastGpsMillis = 0;
uint16_t teamID = 2591;
double maxAltitude = 0;
	
uint32_t pidOnTime = 0;

uint16_t servoManualPos = 500;
	
uint32_t packets = 0;
	
bool pidOn = false;
bool pidJustOn = false;

enum states{UNARMED, PRELAUNCH, ASCENT, DESCENT, ACTIVE, LANDED};

void initialize();

void doCommands();

// Put state transition behavior in these functions
void goToState(uint16_t newState);
void toState0(); // UNARMED
void toState1(); // PRELAUNCH
void toState2(); // ASCENT
void toState3(); // DESCENT
void toState4(); // ACTIVE
void toState5(); // LANDED


int main (void)
{
	initialize();
	
	while (1){
		timekeeper_loop_start();
		
		wdt_reset();
		
		doCommands();
		
		
		if(lastGpsMillis + gpsPeriod < timekeeper_get_millis())
		{
			lastGpsMillis = timekeeper_get_millis();
			gps_update();
			double gpsTime = gps_get_time();
			timekeeper_refine((uint32_t)gpsTime);
		}
		
		imu_update();
		alt_update();
		

		
		double currAlt = alt_get_current_altitude();
		double smooth_altitude = alt_get_smooth_altitude();
		double smooth_velocity = alt_get_smooth_vvel(0.025);
		
		if(lastPacketMillis + telemetryPeriod < timekeeper_get_millis())
		{
			lastPacketMillis = timekeeper_get_millis();
			printf("2591,%lu,%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%lu,ACTIVE,%.0f\n",timekeeper_get_sec(),packets,alt_get_current_altitude()*10,alt_get_pressure(),adc_get_temperature()*10,adc_get_pwr_voltage()*100,gps_get_time(),gps_get_latitude()*100000,gps_get_longitude()*100000,gps_get_altitude()*10,gps_get_sats(),imu_pitch()*10, imu_roll()*10, rpm_get_rate(), imu_heading()*10);
			packets++;
		}
		
		if(lastLogMillis + logPeriod < timekeeper_get_millis())
		{
			lastLogMillis = timekeeper_get_millis();
			//log uses milliseconds instead of seconds since it goes at a higher rate
			log_printf("2591,%lu,%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%lu,ACTIVE,%.0f\n",timekeeper_get_millis(),packets,alt_get_current_altitude()*10,alt_get_pressure(),adc_get_temperature()*10,adc_get_pwr_voltage()*100,gps_get_time(),gps_get_latitude()*100000,gps_get_longitude()*100000,gps_get_altitude()*10,gps_get_sats(),imu_pitch()*10, imu_roll()*10, rpm_get_rate(), imu_heading()*10);
		}
		
		if(pidOn)
		{
			pid_update(imu_heading(),timekeeper_get_millis());
			
			if(pidJustOn)
			{
				pid_clear_integral();
				pidJustOn = false;
			}
			
			fin1_set_pos(pid_output());
			fin2_set_pos(pid_output());
		}
		else
		{	
			fin1_set_pos(servoManualPos);
			fin2_set_pos(servoManualPos);
		}
		
		
		

		if (state == 0){
			//printf("Flight State 0\n");
			
			//if ((smooth_altitude <= 500) && ((int32_t)maxAltitude - (int32_t)smooth_altitude <= -10)){ //Work on Velocity later, this will work for now
			if ((smooth_altitude >= 500) && (smooth_velocity <= -10)){
				state = 1;
				//Turn Camera On
				PORTC.DIRSET = 0b00010000;
				PORTC.OUTSET = 0b00010000;
//				if (timekeeper_get_millis()%600 >= 2) {
//					PORTC.OUTCLR = 0b00010000;
//				}
				delay_ms(600);
				PORTC.OUTCLR = 0b00010000;
			}
			/*if (smooth_altitude > maxAltitude) {
				maxAltitude = smooth_altitude;
 			}*/
		}
		if (state == 1){
			printf("Flight State 1\n");
			
 			if (smooth_altitude<=450){ // Dont need to use initialAltitude, the altimter driver takes care of that
 				state = 2;
 			}
		}
		if (state == 2){
			printf("Flight State 2\n");
 			if ((smooth_altitude <= 50) && (smooth_velocity<=1)){
				state = 3;
				//Turn Camera Off
				PORTC.DIRSET = 0b00010000;
				PORTC.OUTSET = 0b00010000;
				delay_ms(600);
				PORTC.OUTCLR = 0b00010000;
				//Turn Camera On
				PORTC.DIRSET = 0b00010000;
				PORTC.OUTSET = 0b00010000;
				delay_ms(600);
				PORTC.OUTCLR = 0b00010000;
 			}
		}
		if (state == 3){
			printf("Flight State 3\n");
			if ((timekeeper_get_millis()%500) < 250) {
				PORTD.DIRSET = 0b00010000;
				PORTD.OUTSET = 0b00010000;
			}
			else {
				PORTD.OUTCLR = 0b00010000;
			}
			//Buzzer or something
		}
		
		
		timekeeper_loop_end(25);
 	}
}

void doCommands()
{
	if(is_command_ready()){
		char* cmd = get_command();
		char* cmd_prefix = cmd_split(cmd,"/",0);
		printf("CMD RX: %s\n",cmd);
		log_printf("CMD RX: %s\n",cmd);
		if(strcmp(cmd,"CAL_ALT")==0)
		{
			printf("Calibrating altitude\n");
			alt_set_current_to_zero();
		}
		else if(strcmp(cmd,"CAL_IMU")==0)
		{
			printf("Calibrating IMU\n");
			printf("IMU CALIB STAT (0 = uncalibrated and 3 = fully calibrated):\n");
			printf("ACC: %u\n",imu_accel_cal());
			printf("GYR: %u\n",imu_gyro_cal());
			printf("MAG: %u\n",imu_mag_cal());
			printf("SYS: %u\n",imu_sys_cal());
		}
		else if(strcmp(cmd,"RESET")==0)
		{
			printf("Soft Resetting\n");
			wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_4KCLK);
			wdt_enable(); //Enables the watchdog timer if it wasn't already
			while(1); // Waits until the watchdog timer resets the MCU
		}
		else if(strcmp(cmd,"PIDCLR")==0)
		{
			printf("Clearing PID integral\n");
			pid_clear_integral();
		}
		else if(strcmp(cmd_prefix,"PID")== 0)
		{
			printf("Setting PID values\n");
			double kp, ki, kd;
			cmd_parse_pid_values(cmd, &kp, &ki, &kd);
			pid_set_gains(kp, ki, kd);
			printf("kp: %f\nki: %f\nkd:%f\n", kp, ki, kd);
			//TODO add EEPROM write
		}
		else if(strcmp(cmd, "PIDSTART")==0)
		{
			pid_clear_integral();
			pidOn = true;
			pidJustOn = true;
		}
		else if(strcmp(cmd, "PIDSTOP")==0)
		{
			pid_clear_integral();
			pidOn = false;
			fin1_set_pos(500);
			fin2_set_pos(500);
		}
		else if(strcmp(cmd_prefix, "STATE")==0)
		{
			state = cmd_parse_uint16(cmd_split(cmd,"/",1));
			printf("Setting state to %u\n",state);
		}
		else if(strcmp(cmd, "ARM")==0)
		{
			state = 1;
			printf("Setting state to 1");
		}
		else if(strcmp(cmd, "ABORT")==0)
		{
			release_open();
			printf("Opening Release!\n");
		}
		else if(strcmp(cmd, "CLOSE")==0)
		{
			release_close();
			printf("Closing Release!\n");
		}
		else if(strcmp(cmd_prefix, "RATE")==0)
		{
			telemetryPeriod = cmd_parse_uint32(cmd_split(cmd,"/",1));
			printf("Setting rate to %f Hz\n",1000.0 / telemetryPeriod);
		}
		else if(strcmp(cmd_prefix, "FIN")==0)
		{
			servoManualPos = cmd_parse_uint16(cmd_split(cmd,"/",1));
			printf("Setting fin angle to %f\n",1000.0 / servoManualPos);
		}
		else
		{
			printf("Command Unrecognized\n");
		}
		
	}
}

void initialize()
{
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
	log_printf("\n\n\nSECONDARY USART INIT\n"); //This prints to openlog
	
	gps_init(0.0);
	printf("GPS INIT\n");
	log_printf("GPS INIT\n");
	
	timekeeper_init();
	printf("TIMEKEEPER INIT\n");
	log_printf("TIMEKEEPER INIT\n");
	
	rpm_init();
	printf("RPM SENSOR INIT\n");
	log_printf("RPM SENSOR INIT\n");
	
	irq_initialize_vectors();
	cpu_irq_enable();
	
	printf("INTERRUPTS INIT\n");
	log_printf("INTERRUPTS INIT\n");
	
	
	sysclk_enable_peripheral_clock(&TWIC);
	
	sysclk_enable_peripheral_clock(&ADCA);
	adc_init();
	
	printf("ADC INIT\n");
	log_printf("ADC INIT\n");
	
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_4KCLK); // The watchdog timer will reset the MCU if wdt_reset() is not called faster than the timeout period
	wdt_enable();

	wdt_reset();

	imu_init();
	
	printf("IMU INIT\n");
	log_printf("IMU INIT\n");

	wdt_reset();

	alt_init();
	
	printf("ALTIMETER INIT\n");
	log_printf("ALTIMETER INIT\n");
	
	delay_ms(250); //Wait for altitude data before setting zero
	alt_update();
	alt_set_current_to_zero();
	
	delay_ms(20);
	
	printf("SERVO INIT\n");
	log_printf("SERVO INIT\n");
	
	fin1_init(600, 900);
	fin2_init(600, 900);
	release_init(450, 800); //Set these to the actual values needed after we get the servo connector on
	servos_start();
	
	pid_init(10.0, 0.1, 3.0, 180.0); // DEFAULT PID CONSTANTS HERE
	
	printf("\INIT COMPLETE!\n\n\n");
	log_printf("\INIT COMPLETE!\n\n\n");
}
