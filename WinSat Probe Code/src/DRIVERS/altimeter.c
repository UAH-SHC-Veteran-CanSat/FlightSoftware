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

double pres2alt(double pressure);

uint16_t mod(uint16_t a, uint16_t b);

struct bmp280_dev bmp;
struct bmp280_config conf;
struct bmp280_uncomp_data uncomp_data;
double pressure = 0;
double temperature = 0;
double altitude = 0;
double zeroAltitude = 0;

union pres_buffer_element {uint32_t word;};
union pres_buffer_element pres_fifo_buffer[PRES_HIST_BUFFER_LEN];
fifo_desc_t pres_fifo;

union {
	float float_var;
	uint32_t uint_var;
} float_conv;

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
	conf.filter = BMP280_FILTER_COEFF_4;

	/* Pressure oversampling set at 4x */
	conf.os_pres = BMP280_OS_16X;
	
	conf.os_temp = BMP280_OS_2X;

	/* Setting the output data rate as 10HZ(100ms) */
	conf.odr = BMP280_ODR_62_5_MS;
	rslt = bmp280_set_config(&conf, &bmp);
	print_rslt(" bmp280_set_config status", rslt);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	print_rslt(" bmp280_set_power_mode status", rslt);
	
		
	fifo_init(&pres_fifo, pres_fifo_buffer, PRES_HIST_BUFFER_LEN);
}

void alt_set_zero(double zeroAlt)
{
	zeroAltitude = zeroAlt;
}

void alt_set_current_to_zero()
{
	alt_set_zero(pres2alt(pressure));
}


void alt_update()
{
	bmp280_get_uncomp_data(&uncomp_data, &bmp);
	bmp280_get_comp_temp_double(&temperature, uncomp_data.uncomp_temp, &bmp);
	bmp280_get_comp_pres_double(&pressure, uncomp_data.uncomp_press, &bmp);
	
	
	if(fifo_is_full(&pres_fifo))
	{
		uint32_t trashbin;
		fifo_pull_uint32(&pres_fifo, &trashbin);
	}
	float_conv.float_var = pressure;
	fifo_push_uint32(&pres_fifo, float_conv.uint_var);
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
	return pres2alt(pressure) - zeroAltitude;
}

double pres2alt(double pres)
{
	//https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
	
	double p0 = 101325; //Sea level pressure in Pa
	return 44330 * (1-pow((pres/p0),(1.0/5.255)));
}

double alt_get_smooth_altitude()
{
	double average = 0;
	for (uint16_t i = 0; i < fifo_get_used_size(&pres_fifo); i++)
	{
		uint16_t index = (i + pres_fifo.read_index) % PRES_HIST_BUFFER_LEN;
		float_conv.uint_var = pres_fifo_buffer[index].word;
		average += float_conv.float_var/fifo_get_used_size(&pres_fifo);
	}
	return pres2alt(average) - zeroAltitude;
}

//dt is time between calls to alt_update();
double alt_get_current_vvel(double dt)
{;
	uint16_t lastindex = mod(pres_fifo.write_index-2, PRES_HIST_BUFFER_LEN);
	uint16_t index = mod(pres_fifo.write_index-1, PRES_HIST_BUFFER_LEN);
	float_conv.uint_var = pres_fifo_buffer[index].word;
	double this_alt = pres2alt(float_conv.float_var)-zeroAltitude;
	float_conv.uint_var = pres_fifo_buffer[lastindex].word;
	double last_alt = pres2alt(float_conv.float_var)-zeroAltitude;
	
	return (this_alt-last_alt)/(dt*fifo_get_used_size(&pres_fifo));
}

//dt is time between calls to alt_update();
double alt_get_smooth_vvel(double dt)
{
	double average = 0;
	for (uint16_t i = 1; i < fifo_get_used_size(&pres_fifo); i++)
	{
		uint16_t lastindex = (i - 1 + pres_fifo.read_index) % PRES_HIST_BUFFER_LEN;
		uint16_t index = (i + pres_fifo.read_index) % PRES_HIST_BUFFER_LEN;
		float_conv.uint_var = pres_fifo_buffer[index].word;
		double this_alt = pres2alt(float_conv.float_var)-zeroAltitude;
		float_conv.uint_var = pres_fifo_buffer[lastindex].word;
		double last_alt = pres2alt(float_conv.float_var)-zeroAltitude;
		
		average += (this_alt-last_alt);
	}
	return average/(dt*fifo_get_used_size(&pres_fifo));
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

uint16_t mod(uint16_t a, uint16_t b)
{
	int32_t r = (int32_t)a % (int32_t)b;
	return (uint16_t)(r < 0 ? r + b : r);
}
