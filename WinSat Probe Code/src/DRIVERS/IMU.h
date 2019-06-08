/*
 * IMU.h
 *
 * Created: 6/7/2019 11:23:19 AM
 *  Author: trbinsc
 */ 

#include <asf.h>
#include <string.h>
#include "DRIVERS/bno055.h"
#include "libfixmatrix/fixquat.h"


#ifndef IMU_H_
#define IMU_H_

#define FIXMATH_NO_CACHE 1

#define	I2C_BUFFER_LEN 64
#define I2C0 5
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)

void imu_init();

void imu_update();

double imu_accel_x();
double imu_accel_y();
double imu_accel_z();

double imu_roll();
double imu_pitch();
double imu_heading();

uint16_t imu_accel_cal();
uint16_t imu_gyro_cal();
uint16_t imu_mag_cal();
uint16_t imu_sys_cal();



#endif /* IMU_H_ */