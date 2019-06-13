/*
 * EEPROM.c
 *
 * Created: 6/13/2019 7:06:55 AM
 *  Author: trbinsc
 */ 
#include "eeprom.h"

void doubleToBytes(double val, uint8_t* bytes_array);
void uint16ToBytes(uint16_t val, uint8_t* bytes_array);
void uint32ToBytes(uint32_t val, uint8_t* bytes_array);
void int16ToBytes(int16_t val, uint8_t* bytes_array);
void int32ToBytes(int32_t val, uint8_t* bytes_array);

void bytesToDouble(double *val, uint8_t* bytes_array);
void bytesToUint16(uint16_t *val, uint8_t* bytes_array);
void bytesToUint32(uint32_t *val, uint8_t* bytes_array);
void bytesToInt16(int16_t *val, uint8_t* bytes_array);
void bytesToInt32(int32_t *val, uint8_t* bytes_array);

void ee_init()
{
	if(nvm_init(INT_EEPROM) != STATUS_OK)
	{
		printf("NVM INIT FAIL\n");
	}
	ee_write_uint32(65540,0);
	printf("%lu\n",ee_read_uint32(0));
}


void doubleToBytes(double val, uint8_t* bytes_array)
{
	memcpy(bytes_array, (uint8_t*) (&val), 4);
}

void uint16ToBytes(uint16_t val, uint8_t* bytes_array) //bytes_array must be 2 long
{
	memcpy(bytes_array, &val, 2);
}

void uint32ToBytes(uint32_t val, uint8_t* bytes_array) //bytes_array must be 4 long
{
	memcpy(bytes_array, &val, 4);
}

void int16ToBytes(int16_t val, uint8_t* bytes_array) //bytes_array must be 2 long
{
	memcpy(bytes_array, &val, 2);
}

void int32ToBytes(int32_t val, uint8_t* bytes_array) //bytes_array must be 4 long
{
	memcpy(bytes_array, &val, 4);
}

void bytesToDouble(double *val, uint8_t* bytes_array)
{
	memcpy(val, bytes_array, 4);
}

void bytesToUint16(uint16_t *val, uint8_t* bytes_array)
{
	*val = bytes_array[0] | bytes_array[1]<<8;
}

void bytesToUint32(uint32_t *val, uint8_t* bytes_array)
{
	*val = bytes_array[0] | bytes_array[1]<<8 | bytes_array[2]<<16 | bytes_array[3]<<24;
}

void bytesToInt16(int16_t *val, uint8_t* bytes_array)
{
	*val = bytes_array[0] | bytes_array[1]<<8;
}

void bytesToInt32(int32_t *val, uint8_t* bytes_array)
{
	*val = bytes_array[0] | bytes_array[1]<<8 | bytes_array[2]<<16 | bytes_array[3]<<24;
}


void ee_write_uint16(uint16_t data, uint32_t address)
{
	uint8_t buff[2];
	uint16ToBytes(data, buff);
	nvm_write(INT_EEPROM, address, buff, 2);
}

void ee_write_uint32(uint32_t data, uint32_t address)
{
	uint8_t buff[4];
	uint32ToBytes(data, buff);
	nvm_write(INT_EEPROM, address, buff, 4);
}

void ee_write_int16(int16_t data, uint32_t address)
{
	uint8_t buff[2];
	int16ToBytes(data, buff);
	nvm_write(INT_EEPROM, address, buff, 2);
}

void ee_write_int32(int32_t data, uint32_t address)
{
	uint8_t buff[4];
	int32ToBytes(data, buff);
	nvm_write(INT_EEPROM, address, buff, 4);
}

void ee_write_double(double data, uint32_t address)
{
	uint8_t buff[4];
	doubleToBytes(data, buff);
	nvm_write(INT_EEPROM, address, buff, 4);
}


uint16_t ee_read_uint16(uint32_t address)
{
	uint8_t buff[2];
	nvm_read(INT_EEPROM, address, buff, 2);
	uint16_t value;
	bytesToUint16(&value, buff);
	return value;
	
}

uint32_t ee_read_uint32(uint32_t address)
{
	uint8_t buff[4];
	nvm_read(INT_EEPROM, address, buff, 4);
	uint32_t value;
	bytesToUint32(&value, buff);
	return value;
}

int16_t ee_read_int16(uint32_t address)
{
	uint8_t buff[2];
	nvm_read(INT_EEPROM, address, buff, 2);
	int16_t value;
	bytesToInt16(&value, buff);
	return value;
}

int32_t ee_read_int32(uint32_t address)
{
	uint8_t buff[4];
	nvm_read(INT_EEPROM, address, buff, 4);
	int32_t value;
	bytesToInt32(&value, buff);
	return value;
}

double ee_read_double(uint32_t address)
{
	uint8_t buff[4];
	nvm_read(INT_EEPROM, address, buff, 4);
	double value;
	bytesToDouble(&value, buff);
	return value;
}

