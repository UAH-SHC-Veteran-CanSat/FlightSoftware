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
#include "DRIVERS/camAndBuzzer.h"
#include "DRIVERS/eeprom.h"
#include "DRIVERS/dataSaver.h"

#define RESET_TIMEOUT 120 // in seconds

uint16_t state = 0;
uint32_t telemetryPeriod = 1000;
uint32_t logPeriod = 250;
uint32_t gpsPeriod = 1000;
uint32_t lastPacketMillis = 0;
uint32_t lastLogMillis = 0;
uint32_t lastGpsMillis = 0;
uint16_t teamID = 2591;
double maxAltitude = 0;
	
uint16_t servoManualPos = 500;
	
uint32_t packets = 0;
	
bool pidOn = false;
bool pidJustOn = false;
uint32_t pidOnTime = 0;
uint32_t pidOnDelay = 500;

uint16_t next_ee_write = 0;

enum states{UNARMED, PRELAUNCH, ASCENT, DESCENT, ACTIVE, LANDED};
const char* stateNames[6];

uint32_t stateMinTimes[6] = {1000, 1000, 2000, 2000, 90000, 1000}; // Minimum time in milliseconds to spend in each state
uint32_t stateSwitchMillis = 0;

void initialize();
void doCommands();

// Put state transition behavior in these functions
void goToState(uint16_t newState);
void toStateUnarmed(); // UNARMED
void toStatePrelaunch(); // PRELAUNCH
void toStateAscent(); // ASCENT
void toStateDescent(); // DESCENT
void toStateActive(); // ACTIVE
void toStateLanded(); // LANDED

void goToState(uint16_t newState)
{
	switch(newState)
	{
		case UNARMED :
			toStateUnarmed();
			break;
		case PRELAUNCH :
			toStatePrelaunch();
			break;
		case ASCENT :
			toStateAscent();
			break;
		case DESCENT :
			toStateDescent();
			break;
		case ACTIVE :
			toStateActive();
			break;
		case LANDED :
			toStateLanded();
			break;
		default :
			printf("Attempted to enter invalid state\n");
	}
}

void toStateUnarmed() // UNARMED
{
	pidOn = false;
	buz_disable();
	cam_disable();
	maxAltitude = 0;
	if(telemetryPeriod == 30000)
	{
		telemetryPeriod = 1000;
	}
	state = UNARMED;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}

void toStatePrelaunch() //PRELAUNCH
{
	pidOn = false;
	buz_disable();
	cam_disable();
	maxAltitude = 0;
	if(telemetryPeriod == 30000)
	{
		telemetryPeriod = 1000;
	}
	state = PRELAUNCH;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}

void toStateAscent() //ASCENT
{
	pidOn = false;
	buz_disable();
	cam_enable();
	if(telemetryPeriod == 30000)
	{
		telemetryPeriod = 1000;
	}
	state = ASCENT;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}

void toStateDescent() //DESCENT
{
	pidOn = false;
	buz_disable();
	cam_enable();
	if(telemetryPeriod == 30000)
	{
		telemetryPeriod = 1000;
	}
	state = DESCENT;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}

void toStateActive() // ACTIVE
{
	pidOn = true;
	pidOnTime = timekeeper_get_millis() + pidOnDelay;
	pidJustOn = true;
	buz_disable();
	cam_enable();
	release_open();
	if(telemetryPeriod == 30000)
	{
		telemetryPeriod = 1000;
	}
	state = ACTIVE;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}

void toStateLanded() // LANDED
{
	pidOn = false;
	buz_enable();
	cam_disable();
	telemetryPeriod = 30000;
	lastPacketMillis = 0;
	state = LANDED;
	stateSwitchMillis = timekeeper_get_millis();
	save_state(state);
}


int main (void)
{
	stateNames[0] = "UNARMED";
	stateNames[1] = "PRELAUNCH";
	stateNames[2] = "ASCENT";
	stateNames[3] = "DESCENT";
	stateNames[4] = "ACTIVE";
	stateNames[5] = "LANDED";
	
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
		
		if(pidOn && timekeeper_get_millis() > pidOnTime)
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
		
		//double currAlt = alt_get_current_altitude(); currently unused, commented out for speed
		double smooth_altitude = alt_get_smooth_altitude();
		double smooth_velocity = alt_get_smooth_vvel(0.05);
		
		if(next_ee_write == 1)
		{
			save_packets(packets);
			next_ee_write = 2;
		}
		else if(next_ee_write == 2)
		{
			save_time(timekeeper_get_sec());
			next_ee_write = 3;
		}
		else if(next_ee_write == 3)
		{
			save_utc((uint32_t)gps_get_time());
			next_ee_write = 0;
		}
		
		if(lastPacketMillis + telemetryPeriod < timekeeper_get_millis())
		{
			lastPacketMillis = timekeeper_get_millis();
			printf("2591,%lu,%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%lu,%s,%.0f\n",timekeeper_get_sec(),packets,smooth_altitude*10,alt_get_pressure(),adc_get_temperature()*10,adc_get_pwr_voltage()*100,gps_get_time(),gps_get_latitude()*100000,gps_get_longitude()*100000,gps_get_altitude()*10,gps_get_sats(),imu_pitch()*10, imu_roll()*10, rpm_get_rate(), stateNames[state], imu_heading()*10);
			packets++;
			next_ee_write = 1;
		}
		
		if(lastLogMillis + logPeriod < timekeeper_get_millis())
		{
			lastLogMillis = timekeeper_get_millis();
			//log uses milliseconds instead of seconds since it goes at a higher rate
			log_printf("2591,%lu,%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%lu,%s,%.0f\n",timekeeper_get_millis(),packets,smooth_altitude*10,alt_get_pressure(),adc_get_temperature()*10,adc_get_pwr_voltage()*100,gps_get_time(),gps_get_latitude()*100000,gps_get_longitude()*100000,gps_get_altitude()*10,gps_get_sats(),imu_pitch()*10, imu_roll()*10, rpm_get_rate(), stateNames[state],imu_heading()*10);
		}
		
		buz_update(timekeeper_get_millis());
		cam_update(timekeeper_get_millis());
		


		if (state == UNARMED)
		{
			//Don't do anything but wait to be armed
		}
		else if (state == PRELAUNCH)
		{
			if(smooth_altitude > 50 || smooth_velocity > 20 || (smooth_altitude>20 && smooth_velocity > 10))
			{
				if(timekeeper_get_millis() > stateSwitchMillis + stateMinTimes[state])
				{
					toStateAscent();
				}
				
			}
		}
		else if (state == ASCENT)
		{
			if(smooth_altitude > maxAltitude)
			{
				maxAltitude = smooth_altitude;
			}
			
			if(smooth_altitude < maxAltitude - 20)
			{
				if(timekeeper_get_millis() > stateSwitchMillis + stateMinTimes[state])
				{
					toStateDescent();
				}
			}
		}
		else if (state == DESCENT)
		{
			if(smooth_altitude < 460)
			{
				if(timekeeper_get_millis() > stateSwitchMillis + stateMinTimes[state])
				{
					toStateActive();
				}
			}
		}
		else if (state == ACTIVE)
		{
			if(smooth_altitude < 20 || abs(smooth_velocity) < 4)
			{
				if(timekeeper_get_millis() > stateSwitchMillis + stateMinTimes[state])
				{
					toStateLanded();
				}
			}
		}
		else if (state == LANDED)
		{
			//Woohoo we did it
		}
		
		
		
		
		timekeeper_loop_end(50);
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
			gps_zero_current_alt();
			save_ground_alt(alt_get_zero());
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
		else if(strcmp(cmd,"HARD_RESET")==0)
		{
			printf("Hard Resetting\n");
			save_state(UNARMED);
			save_packets(0);
			save_time(0);
			save_utc(0);
			save_ground_alt(0);
			
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
			pidOnTime = timekeeper_get_millis() + pidOnDelay;
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
			goToState(cmd_parse_uint16(cmd_split(cmd,"/",1)));
			printf("Setting state to %s\n",stateNames[state]);
		}
		else if(strcmp(cmd, "ARM")==0)
		{
			goToState(PRELAUNCH);
			printf("Arming CanSat!\n");
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
		else if(strcmp(cmd, "CAMON")==0)
		{
			cam_enable();
			printf("Starting Camera\n");
		}
		else if(strcmp(cmd, "CAMOFF")==0)
		{
			cam_disable();
			printf("Stopping Camera\n");
		}
		else if(strcmp(cmd_prefix, "RATE")==0)
		{
			telemetryPeriod = cmd_parse_uint32(cmd_split(cmd,"/",1));
			printf("Setting rate to %f Hz\n",1000.0 / telemetryPeriod);
		}
		else if(strcmp(cmd_prefix, "FIN")==0)
		{
			servoManualPos = cmd_parse_uint16(cmd_split(cmd,"/",1));
			printf("Setting fin position to %f%%\n",servoManualPos/1000.0);
		}
		else if(strcmp(cmd, "Hi CanSat, how are you?")==0 || strcmp(cmd, "Hi CanSat")==0)
		{
			if(state == UNARMED)
			{
				printf("I'm kinda bored, just waiting to be told to do something.\n");
			}
			else if(state == PRELAUNCH)
			{
				printf("I'm excited, I can't wait to launch!\n");
			}
			else if(state == ASCENT)
			{
				printf("WEEEEEEEEEEEEE!!!\n");
			}
			else if(state == DESCENT)
			{
				printf("WEEEOOooooOOOOAAAAHHHH!!\n");
			}
			else if(state == ACTIVE)
			{
				printf("Gotta... concentrate... on... pointing...\n");
			}
			else if(state == LANDED)
			{
				printf("Whew, that was great, I want to go again!\n");
			}
		}
		else
		{
			printf("Command Unrecognized\n");
		}
		//TODO: Hard Reset that clears eeprom
		
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
	
	cambuz_init();
	
	ee_init();
	
	printf("\nINIT COMPLETE!\n\n\n");
	log_printf("\nINIT COMPLETE!\n\n\n");
	
	printf("CHECKING RESET STATUS\n");
	
	wdt_reset();
	if (read_state() != UNARMED)
	{
		printf("LAST STATE NOT UNARMED %u\n");
		uint32_t resetCheckStart = timekeeper_get_millis();
		while(timekeeper_get_millis() < resetCheckStart + 5000)
		{
			wdt_reset();
			gps_update();
			
			if(gps_get_time() != 0)
			{
				break;
			}
			delay_ms(1000);
		}
		
		if(gps_get_time() == 0)
		{
			printf("UNABLE TO INITIALIZE GPS\nCANNOT DETERMINE PREVIOUS STATE\n");
			toStateUnarmed();
		}
		else if((uint32_t) gps_get_time() < read_utc()+RESET_TIMEOUT)
		{
			printf("LAST STATE INSIDE TIMEOUT!\nRESUMING\n!");
			uint32_t diffMills = ((uint32_t)gps_get_time() - read_utc())*1000;
			timekeeper_set_millis(read_time()*1000 + diffMills);
			packets = read_packets();
			alt_set_zero(read_ground_alt());
			uint16_t newState = read_state();
			printf("ADVANCING TO STATE %s\n",stateNames[newState]);
			goToState(newState);
		}
		else
		{
			printf("LAST STATE OUTSIDE TIMEOUT, IGNORING\n");
			toStateUnarmed();
		}
	}
	else
	{
		printf("PREVIOUS STATE NOT DETECTED\n");
		toStateUnarmed();
	}
	
}
