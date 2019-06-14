/*
 * IMU.c
 *
 * Created: 6/7/2019 11:22:59 AM
 *  Author: trbinsc
 */ 

#include "IMU.h"

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

struct bno055_t bno055; 
struct bno055_linear_accel_double_t bno055_linear_accel;
struct bno055_euler_double_t bno055_euler;

uint8_t accel_calib;
uint8_t gyro_calib;
uint8_t mag_calib;
uint8_t sys_calib;

uint16_t uncalib_count = 0;

void imu_init()
{
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};

	sysclk_enable_peripheral_clock(&IMU_TWI);
	twi_master_init(&IMU_TWI, &m_options);
	twi_master_enable(&IMU_TWI);

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR2;


	int8_t initStatus = bno055_init(&bno055);
	if (initStatus != 0)
	{
		printf("BNO55 INIT STATUS: %i  (0 is good)\n", initStatus);
	}
	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	int8_t status = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	uint8_t operation_mode = 0;
	bno055_get_operation_mode(&operation_mode);
	if(operation_mode != BNO055_OPERATION_MODE_NDOF)
	{
		printf("BNO055 Operation mode is %u, should be %u\nWrite status: %i  (0 is good)\n",operation_mode,BNO055_OPERATION_MODE_NDOF,status);
	}
	


	
}

void imu_update()
{
	bno055_convert_double_linear_accel_xyz_msq(&bno055_linear_accel);
	bno055_convert_double_euler_hpr_deg(&bno055_euler);
	
	bno055_get_accel_calib_stat(&accel_calib);
	bno055_get_gyro_calib_stat(&gyro_calib);
	bno055_get_mag_calib_stat(&mag_calib);
	bno055_get_sys_calib_stat(&sys_calib);
	
	if (accel_calib == 0 && mag_calib == 0 && gyro_calib == 0 && sys_calib == 0)
	{
		//printf("IMU Completely uncalibrated\n");
		uncalib_count++;
	}
}


double imu_accel_x()
{
	return bno055_linear_accel.x;
}

double imu_accel_y()
{
	return bno055_linear_accel.y;
}

double imu_accel_z()
{
	return bno055_linear_accel.z;
}



double imu_roll()
{
	return bno055_euler.r;
}

double imu_pitch()
{
	return bno055_euler.p;
}

double imu_heading()
{
	return bno055_euler.h;
}


uint8_t imu_accel_cal()
{
	return accel_calib;
}

uint8_t imu_gyro_cal()
{
	return gyro_calib;
}

uint8_t imu_mag_cal()
{
	return mag_calib;
}

uint8_t imu_sys_cal()
{
	return sys_calib;
}

uint32_t imu_uncalib_count()
{
	return uncalib_count;
}



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


	BNO055_iERROR = (s8)twi_master_write(&IMU_TWI,&readbno055);

// 	printf("I2C Write cnt=%u status=%i Data:  ",cnt,BNO055_iERROR);
// 	for (u8 i = 0; i < cnt; i++)
// 	{
// 		printf(" %x, ",array[i]);
// 	}
// 	printf("\n");

	return (s8)BNO055_iERROR;
}

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

	BNO055_iERROR = (int8_t) twi_master_read(&IMU_TWI, &readbno055);
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