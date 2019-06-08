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
#include "DRIVERS/altitude.h"
//#include "DRIVERS/pressure.h"
#include "DRIVERS/temperature.h"
#include "DRIVERS/IMU.h"


//Thomas's Code -------------------------------------------------------------------------------------------------
void print_rslt(const char api_name[], int8_t rslt);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{


	twi_package_t readbmp280;
	readbmp280.addr[0]	   = reg_addr;
	readbmp280.addr_length = 1;
	readbmp280.chip        = i2c_addr;
	readbmp280.buffer      = reg_data;
	readbmp280.length      = length;
	readbmp280.no_wait     = false; 

	
	int8_t result = twi_master_write(&TWIC, &readbmp280);
	
	return result;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    uint8_t array[I2C_BUFFER_LEN] = {0};
    
    twi_package_t readbmp280;
    readbmp280.addr[0]	   = reg_addr;
    readbmp280.addr_length = 1;
    readbmp280.chip        = i2c_addr;
    readbmp280.buffer      = array;
    readbmp280.length      = length;
    readbmp280.no_wait     = false;
    
    int8_t result = twi_master_read(&TWIC, &readbmp280);
    memcpy(reg_data, array, length);
    
    return result;
}

// struct bmp280_dev bmp;
// struct bmp280_config conf;
// struct bmp280_uncomp_data ucomp_data;
// uint32_t pres32, pres64;
// double pres;
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

// 	printf("Starting BMP280 Init\n");
// 	
// 	bmp.read = i2c_reg_read;
// 	bmp.write = i2c_reg_write;
// 	bmp.delay_ms = BNO055_delay_msek;
// 	
// 	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
// 	bmp.intf = BMP280_I2C_INTF;
// 	
// 	int8_t rslt;
// 	
// 	delay_ms(500);
// 	
// 	rslt = bmp280_init(&bmp);
// 	print_rslt(" bmp280_init status", rslt);
// 	
// 	rslt = bmp280_get_config(&conf, &bmp);
// 	print_rslt(" bmp280_get_config status", rslt);
// 	
// 	/* configuring the temperature oversampling, filter coefficient and output data rate */
// 	/* Overwrite the desired settings */
// 	conf.filter = BMP280_FILTER_COEFF_2;
// 
// 	/* Pressure oversampling set at 4x */
// 	conf.os_pres = BMP280_OS_8X;
// 
// 	/* Setting the output data rate as 1HZ(1000ms) */
// 	conf.odr = BMP280_ODR_1000_MS;
// 	rslt = bmp280_set_config(&conf, &bmp);
// 	print_rslt(" bmp280_set_config status", rslt);
// 
// 	/* Always set the power mode after setting the configuration */
// 	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
// 	print_rslt(" bmp280_set_power_mode status", rslt);

	printf("\nInitialization Complete!\n\n\n");
	printf("Testing float print %f\n",420.69);
	
	delay_ms(500);
	
	
// 	v3d position;
// 	position.x = 0;
// 	position.y = 0;
// 	position.z = 0;
// 	v3d velocity;
// 	velocity.x = 0;
// 	velocity.y = 0;
// 	velocity.z = 0;
// 	v3d acceleration;
// 	
// 	qf16 orientation;
// 
// 	uint32_t last_time = 0;
// 	
// 	/* Insert application code here, after the board has been initialized. */
// 	uint8_t state = 0;
// 	uint8_t teamID = 2591;
// 	uint32_t initialPressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
// 	uint32_t pressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
// 	int32_t temperature = getTemperature();
// 	int32_t initialAltitude = getAltitude(initialPressure,pressure,temperature);
// 	int32_t maxAltitude = 0;
// 	int32_t altitude = 0;
// 	//int32_t velocity = 0;
// 	int32_t smooth_altitude = 0;
// 	double smoothing_factor = 0.50;
// 	int32_t altitudeArray[10];
	
	
	while (1){
		
//		printf("hi\n");
		delay_ms(100);
		if(is_command_ready()){
			printf("%s\n",get_command());
		}
		gps_update();
// 		printf("time:%f\n",gps_get_time());
// 		printf("lat :%f\n",gps_get_latitude());
// 		printf("lon :%f\n",gps_get_longitude());
// 		printf("alt :%f\n",gps_get_altitude());
// 		printf("sats:%u\n\n",gps_get_sats());
		
/*		printf("temperature: %f\n\n",getTemperature());*/
		
		imu_update();
		
		printf("2591,0,0,0,0,0,0,0,0,0,0,0,%.0f,%.0f,00,PRELAUNCH,%.0f\n",imu_pitch(), imu_roll(), imu_heading());
		//printf("CALBRATION STATUSES:  Accel: %u, Gyro: %u, Mag: %u, Sys: %u\n", imu_accel_cal(), imu_gyro_cal(), imu_mag_cal(), imu_sys_cal());
		
// 		printf("roll : %f\n",imu_roll());
// 		printf("pitch: %f\n",imu_pitch());
// 		printf("head : %f\n",imu_heading());

// 		struct bno055_linear_accel_t bno055_linear_accel;
// 		bno055_read_linear_accel_xyz(&bno055_linear_accel);
// 		acceleration.x = fix16_from_float((float)bno055_linear_accel.x / 100.0);
// 		acceleration.y = fix16_from_float((float)bno055_linear_accel.y / 100.0);
// 		acceleration.z = fix16_from_float((float)bno055_linear_accel.z / 100.0);
// 
// 		struct bno055_quaternion_t bno055_quaternion;
// 		bno055_read_quaternion_wxyz(&bno055_quaternion);
// 		orientation.a = fix16_from_int(bno055_quaternion.w);
// 		orientation.b = fix16_from_int(bno055_quaternion.x);
// 		orientation.c = fix16_from_int(bno055_quaternion.y);
// 		orientation.d = fix16_from_int(bno055_quaternion.z);
// 
// 		// Auto reset if BNO055 stops working
// 		// 		if(bno055_quaternion.w == 0 && bno055_quaternion.x == 0 && bno055_quaternion.y == 0 && bno055_quaternion.z == 0 && bno055_linear_accel.x == 0 && bno055_linear_accel.y == 0 && bno055_linear_accel.z == 0)
// 		// 		{
// 		// 			int8_t initStatus = bno055_init(&bno055);
// 		// 			printf("Init Status: %i  (0 is good)\n", initStatus);
// 		// 			bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
// 		// 			//bno055_set_clk_src(0x01);
// 		// 			int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
// 		// 			uint8_t operation_mode = 0;
// 		// 			bno055_get_operation_mode(&operation_mode);
// 		// 			printf("Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);
// 		//
// 		// 			printf("IMU Initialized\n");
// 		// 			delay_ms(500);
// 		// 		}
// 		
// 		uint8_t accel_calib = 0;
// 		uint8_t gyro_calib = 0;
// 		uint8_t mag_calib = 0;
// 		uint8_t sys_calib = 0;
// 		bno055_get_accel_calib_stat(&accel_calib);
// 		bno055_get_gyro_calib_stat(&gyro_calib);
// 		bno055_get_mag_calib_stat(&mag_calib);
// 		bno055_get_sys_calib_stat(&sys_calib);
// 		
// 		qf16_normalize(&orientation, &orientation);
// 		qf16_conj(&orientation, &orientation); //Inverts quaternion (inverse of unit quaternion is the conjugate)
// 
// 
// 		
// 		printf("%i, %i, %i, %i, %i, %i, %i\n",
// 		bno055_quaternion.w, bno055_quaternion.x, bno055_quaternion.y, bno055_quaternion.z, bno055_linear_accel.x, bno055_linear_accel.y, bno055_linear_accel.z);
// 		
// 		if (accel_calib > 1 && gyro_calib > 1 && mag_calib > 1 && sys_calib > 1)
// 		{
// 			
// 		}
// 		else
// 		{
// 			printf("Calib stat: %u %u %u %u\n",accel_calib, gyro_calib, mag_calib, sys_calib);
// 		}
// 		
// 		// 				printf("%lu,",cycles);
// 		// 				printf("%li,%lu,%lu,",pressure,temperature);
// 		// 				//printf("%.3f,%.3f,%.3f,%.3f,",yaw_to_heading(imu_data.yaw),imu_data.yaw,imu_data.roll,imu_data.pitch);
// 		// 				//printf("%.3f,%i,",yawAngle,pitchAngle);
// 		// 				printf("%.5f,%.5f,",GPSdata.latdecimal,GPSdata.londecimal);
// 		
// 		bmp280_get_uncomp_data(&ucomp_data, &bmp);
// 		bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);
// 		bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
// 		bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
// 		
// 		print_rslt("get_uncomp_data",rslt);
// 		printf("%lu\n",pres32);
// 		
// 		pressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
// 		altitude = getAltitude(initialPressure, pressure, temperature);
// 		//velocity = getVelocity();
// 		smooth_altitude = (int32_t)(smoothing_factor * altitude + (1-smoothing_factor)*smooth_altitude);
// 		
// 		//Velocity Tentative Idea
// 		altitudeArray[0] = smooth_altitude;
// 		for (int n = 1; n < 9; n++){
// 			altitudeArray[n] = altitudeArray[n-1];
// 		}
// 		
// 		//teamID,my_time,packetCount,altitude,pressure,temperature,voltage,GPSTime,GPSLat,GPSLong,GPSAlt,GPSSats,smooth_x,smooth_y,smooth_z,state
// 		
// 		if (state == 0){
// 			printf("Flight State 0");
// 			if ((smooth_altitude <= 500) && ((int32_t)maxAltitude - (int32_t)smooth_altitude <= 0)){ //Work on Velocity later, this will work for now
// 				state = 1;
// 			}
// 			if (smooth_altitude > maxAltitude) {
// 				maxAltitude = smooth_altitude;
// 			}
// 		}
// 		if (state == 1){
// 			printf("Flight State 1");
// 			if (smooth_altitude-initialAltitude<=450){
// 				state = 2;
// 			}
// 		}
// 		if (state == 2){
// 			printf("Flight State 2");
// 			if ((smooth_altitude - initialAltitude <= 50) /*&& (velocity<=1)*/){
// 				state = 3;
// 			}
// 		}
// 		if (state == 3){
// 			printf("Flight State 3");
// 			//Buzzer or something
// 		}
 	}
}

// void print_rslt(const char api_name[], int8_t rslt)
// {
// 	if (rslt != BMP280_OK)
// 	{
// 		printf("%s\t", api_name);
// 		if (rslt == BMP280_E_NULL_PTR)
// 		{
// 			printf("Error [%d] : Null pointer error\r\n", rslt);
// 		}
// 		else if (rslt == BMP280_E_COMM_FAIL)
// 		{
// 			printf("Error [%d] : Bus communication failed\r\n", rslt);
// 		}
// 		else if (rslt == BMP280_E_IMPLAUS_TEMP)
// 		{
// 			printf("Error [%d] : Invalid Temperature\r\n", rslt);
// 		}
// 		else if (rslt == BMP280_E_DEV_NOT_FOUND)
// 		{
// 			printf("Error [%d] : Device not found\r\n", rslt);
// 		}
// 		else
// 		{
// 			/* For more error codes refer "*_defs.h" */
// 			printf("Error [%d] : Unknown error code\r\n", rslt);
// 		}
// 	}
// }
