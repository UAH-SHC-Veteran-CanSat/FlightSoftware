/*
 * altimeter.c
 *
 * Created: 6/8/2019 7:38:04 PM
 *  Author: trbinsc
 */ 

#include "DRIVERS/altimeter.h"

void print_rslt(const char api_name[], int8_t rslt);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void bmp280_delay_msek(uint32_t msek);

struct bmp280_dev bmp;
struct bmp280_config conf;
struct bmp280_uncomp_data uncomp_data;
double pressure = 0;
double temperature = 0;
double altitude = 0;
double zeroAltitude = 0;
double vVel = 0;


void alt_init()
{
	
	twi_options_t m_options = {
		.speed = 400000,
		.speed_reg = TWI_BAUD(32000000, 400000),
	};
	
	sysclk_enable_peripheral_clock(&ALT_TWI);
	twi_master_init(&ALT_TWI, &m_options);
	twi_master_enable(&ALT_TWI);

	bmp.read = i2c_reg_read;
	bmp.write = i2c_reg_write;
	bmp.delay_ms = bmp280_delay_msek;

	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
	bmp.intf = BMP280_I2C_INTF;

	int8_t rslt;
	
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
	conf.odr = BMP280_ODR_125_MS;
	rslt = bmp280_set_config(&conf, &bmp);
	print_rslt(" bmp280_set_config status", rslt);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	print_rslt(" bmp280_set_power_mode status", rslt);
}

void alt_set_zero(double zeroAlt)
{
	zeroAltitude = zeroAlt;
}


void alt_update()
{
	bmp280_get_uncomp_data(&uncomp_data, &bmp);
	bmp280_get_comp_temp_double(&temperature, uncomp_data.uncomp_temp, &bmp);
	bmp280_get_comp_pres_double(&pressure, uncomp_data.uncomp_press, &bmp);
}


double alt_get_pressure()
{
	return pressure;
}

double alt_get_temperature()
{
	return temperature;
}

double alt_get_current_altitude()
{
	
}

double alt_get_smooth_altitude()
{
	
}

double alt_get_current_vvel()
{
	
}

double alt_get_smooth_vvel()
{
	
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

	
	int8_t result = twi_master_write(&ALT_TWI, &readbmp280);
	
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
    
    int8_t result = twi_master_read(&ALT_TWI, &readbmp280);
    memcpy(reg_data, array, length);
    
    return result;
}

void bmp280_delay_msek(uint32_t msek)
{
	delay_ms(msek);
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