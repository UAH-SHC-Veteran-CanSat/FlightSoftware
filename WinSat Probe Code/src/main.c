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
#include <stdio.h>
#include "DRIVERS/altitude.h"
//#include "DRIVERS/pressure.h"
#include "DRIVERS/temperature.h"
#include "DRIVERS/bmp280.h"
#include "DRIVERS/bno055.h"
#include "libfixmatrix/fixquat.h"


//Thomas's Code -------------------------------------------------------------------------------------------------
void print_rslt(const char api_name[], int8_t rslt);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

#define FIXMATH_NO_CACHE 1

#define	I2C_BUFFER_LEN 64
#define I2C0 5
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
// 	u8 stringpos = BNO055_INIT_VALUE;


// 	array[BNO055_INIT_VALUE] = reg_addr;
// 	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
// 	{
// 		array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
// 			*(reg_data + stringpos);
// 	}

	if(cnt > 1)
	{
		printf("The BNO055 Actually does write more than one byte at a time, isn't that surprising.\nI guess it's time to fix the I2C write hack then.");
	}

	cnt = cnt + 1;	// BNO055 Discards the first write, so we make the first value 0
	array[0] = 0;
	array[1] = reg_data[0]; // This breaks if it ever sends more than one byte at a time, but I don't think it does;

	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/

	

	twi_package_t readbno055;
	readbno055.addr[0]	   = reg_addr-1; // it's minus one because the BNO055 discards the first write, and the second write one is at the next address 
	readbno055.addr_length = 1;
	readbno055.chip        = dev_addr;
	readbno055.buffer      = array;
	readbno055.length      = cnt;
	readbno055.no_wait     = false;


	BNO055_iERROR = (s8)twi_master_write(&TWIC,&readbno055);

// 	printf("I2C Write cnt=%u status=%i Data:  ",cnt,BNO055_iERROR);
// 	for (u8 i = 0; i < cnt; i++)
// 	{
// 		printf(" %x, ",array[i]);
// 	}
// 	printf("\n");

	return (s8)BNO055_iERROR;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
// 	u8 stringpos = BNO055_INIT_VALUE;
// 
// 	array[BNO055_INIT_VALUE] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * BNO055_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */

	twi_package_t readbno055;
	readbno055.addr[0]	   = reg_addr;
	readbno055.addr_length = 1;
	readbno055.chip        = dev_addr;
	readbno055.buffer      = array;
	readbno055.length      = cnt;
	readbno055.no_wait     = false;

	BNO055_iERROR = (int8_t) twi_master_read(&TWIC, &readbno055);
	memcpy(reg_data, array, cnt);
	
/*	printf("I2C Read: %x\n",array[0]);*/

// 	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
// 		*(reg_data + stringpos) = array[stringpos];
	return (s8)BNO055_iERROR;
}

void BNO055_delay_msek(u32 msek)
{
	delay_ms(msek);
}

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

struct bno055_t bno055; 
struct bmp280_dev bmp;
struct bmp280_config conf;
struct bmp280_uncomp_data ucomp_data;
uint32_t pres32, pres64;
double pres;
//Thomas' Code---------------------------------------------------------------------------------------------------

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();

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
	
	sysclk_enable_peripheral_clock(&TWIC);
	
	sysclk_enable_peripheral_clock(&ADCA);
	adc_init();
	
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_1KCLK);
	//wdt_enable();
	
	pmic_init();
	irq_initialize_vectors();
	cpu_irq_enable();
	
	printf("PMIC Initialized\n");
	

	TCE0.CTRLA = 0b00000110;
	TCE0.PER = 121;
	TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	wdt_reset();


	printf("Starting BNO055 Init\n");
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};
	
	sysclk_enable_peripheral_clock(&TWIC);
	twi_master_init(&TWIC, &m_options);
	twi_master_enable(&TWIC);

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR2;

	
	int8_t initStatus = bno055_init(&bno055);
	printf("Init Status: %i  (0 is good)\n", initStatus);
	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	//bno055_set_clk_src(0x01);
	int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	uint8_t operation_mode = 0;
	bno055_get_operation_mode(&operation_mode);
	printf("Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);
	
	wdt_reset();
	
	printf("IMU Initialized\n");
	
	printf("Starting BMP280 Init\n");
	
	bmp.read = i2c_reg_read;
	bmp.write = i2c_reg_write;
	bmp.delay_ms = BNO055_delay_msek;
	
	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
	bmp.intf = BMP280_I2C_INTF;
	
	int8_t rslt;
	
	delay_ms(500);
	
	rslt = bmp280_init(&bmp);
	print_rslt(" bmp280_init status", rslt);
	
	rslt = bmp280_get_config(&conf, &bmp);
	print_rslt(" bmp280_get_config status", rslt);
	
	/* configuring the temperature oversampling, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_COEFF_2;

	/* Pressure oversampling set at 4x */
	conf.os_pres = BMP280_OS_8X;

	/* Setting the output data rate as 1HZ(1000ms) */
	conf.odr = BMP280_ODR_1000_MS;
	rslt = bmp280_set_config(&conf, &bmp);
	print_rslt(" bmp280_set_config status", rslt);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	print_rslt(" bmp280_set_power_mode status", rslt);

	printf("\nInitialization Complete!\n\n\n");
	
	
	v3d position;
	position.x = 0;
	position.y = 0;
	position.z = 0;
	v3d velocity;
	velocity.x = 0;
	velocity.y = 0;
	velocity.z = 0;
	v3d acceleration;
	
	qf16 orientation;

	uint32_t last_time = 0;
	
	/* Insert application code here, after the board has been initialized. */
	uint8_t state = 0;
	uint8_t teamID = 2591;
	uint32_t initialPressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
	uint32_t pressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
	int32_t temperature = getTemperature();
	int32_t initialAltitude = getAltitude(initialPressure,pressure,temperature);
	int32_t maxAltitude = 0;
	int32_t altitude = 0;
	//int32_t velocity = 0;
	int32_t smooth_altitude = 0;
	double smoothing_factor = 0.50;
	int32_t altitudeArray[10];
	
	
	while (1){
		struct bno055_linear_accel_t bno055_linear_accel;
		bno055_read_linear_accel_xyz(&bno055_linear_accel);
		acceleration.x = fix16_from_float((float)bno055_linear_accel.x / 100.0);
		acceleration.y = fix16_from_float((float)bno055_linear_accel.y / 100.0);
		acceleration.z = fix16_from_float((float)bno055_linear_accel.z / 100.0);

		struct bno055_quaternion_t bno055_quaternion;
		bno055_read_quaternion_wxyz(&bno055_quaternion);
		orientation.a = fix16_from_int(bno055_quaternion.w);
		orientation.b = fix16_from_int(bno055_quaternion.x);
		orientation.c = fix16_from_int(bno055_quaternion.y);
		orientation.d = fix16_from_int(bno055_quaternion.z);

		// Auto reset if BNO055 stops working
		// 		if(bno055_quaternion.w == 0 && bno055_quaternion.x == 0 && bno055_quaternion.y == 0 && bno055_quaternion.z == 0 && bno055_linear_accel.x == 0 && bno055_linear_accel.y == 0 && bno055_linear_accel.z == 0)
		// 		{
		// 			int8_t initStatus = bno055_init(&bno055);
		// 			printf("Init Status: %i  (0 is good)\n", initStatus);
		// 			bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
		// 			//bno055_set_clk_src(0x01);
		// 			int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
		// 			uint8_t operation_mode = 0;
		// 			bno055_get_operation_mode(&operation_mode);
		// 			printf("Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);
		//
		// 			printf("IMU Initialized\n");
		// 			delay_ms(500);
		// 		}
		
		uint8_t accel_calib = 0;
		uint8_t gyro_calib = 0;
		uint8_t mag_calib = 0;
		uint8_t sys_calib = 0;
		bno055_get_accel_calib_stat(&accel_calib);
		bno055_get_gyro_calib_stat(&gyro_calib);
		bno055_get_mag_calib_stat(&mag_calib);
		bno055_get_sys_calib_stat(&sys_calib);
		
		qf16_normalize(&orientation, &orientation);
		qf16_conj(&orientation, &orientation); //Inverts quaternion (inverse of unit quaternion is the conjugate)


		
		printf("%i, %i, %i, %i, %i, %i, %i\n",
		bno055_quaternion.w, bno055_quaternion.x, bno055_quaternion.y, bno055_quaternion.z, bno055_linear_accel.x, bno055_linear_accel.y, bno055_linear_accel.z);
		
		if (accel_calib > 1 && gyro_calib > 1 && mag_calib > 1 && sys_calib > 1)
		{
			
		}
		else
		{
			printf("Calib stat: %u %u %u %u\n",accel_calib, gyro_calib, mag_calib, sys_calib);
		}
		
		// 				printf("%lu,",cycles);
		// 				printf("%li,%lu,%lu,",pressure,temperature);
		// 				//printf("%.3f,%.3f,%.3f,%.3f,",yaw_to_heading(imu_data.yaw),imu_data.yaw,imu_data.roll,imu_data.pitch);
		// 				//printf("%.3f,%i,",yawAngle,pitchAngle);
		// 				printf("%.5f,%.5f,",GPSdata.latdecimal,GPSdata.londecimal);
		
		bmp280_get_uncomp_data(&ucomp_data, &bmp);
		bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);
		bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
		bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
		
		print_rslt("get_uncomp_data",rslt);
		printf("%lu\n",pres32);
		
		pressure = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);
		altitude = getAltitude(initialPressure, pressure, temperature);
		//velocity = getVelocity();
		smooth_altitude = (int32_t)(smoothing_factor * altitude + (1-smoothing_factor)*smooth_altitude);
		
		//Velocity Tentative Idea
		altitudeArray[0] = smooth_altitude;
		for (int n = 1; n < 9; n++){
			altitudeArray[n] = altitudeArray[n-1];
		}
		
		//teamID,my_time,packetCount,altitude,pressure,temperature,voltage,GPSTime,GPSLat,GPSLong,GPSAlt,GPSSats,smooth_x,smooth_y,smooth_z,state
		
		if (state == 0){
			printf("Flight State 0");
			if ((smooth_altitude <= 500) && ((int32_t)maxAltitude - (int32_t)smooth_altitude <= 0)){ //Work on Velocity later, this will work for now
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
			if ((smooth_altitude - initialAltitude <= 50) /*&& (velocity<=1)*/){
				state = 3;
			}
		}
		if (state == 3){
			printf("Flight State 3");
			//Buzzer or something
		}
	}
}

void print_rslt(const char api_name[], int8_t rslt)
{
	if (rslt != BMP280_OK)
	{
		printf("%s\t", api_name);
		if (rslt == BMP280_E_NULL_PTR)
		{
			printf("Error [%d] : Null pointer error\r\n", rslt);
		}
		else if (rslt == BMP280_E_COMM_FAIL)
		{
			printf("Error [%d] : Bus communication failed\r\n", rslt);
		}
		else if (rslt == BMP280_E_IMPLAUS_TEMP)
		{
			printf("Error [%d] : Invalid Temperature\r\n", rslt);
		}
		else if (rslt == BMP280_E_DEV_NOT_FOUND)
		{
			printf("Error [%d] : Device not found\r\n", rslt);
		}
		else
		{
			/* For more error codes refer "*_defs.h" */
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	}
}
