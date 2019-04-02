/*
 * pressure.c
 *
 * Created: 3/27/2019 2:53:46 PM
 *  Author: Mason
 */ 

int32_t t_fine;
int32_t getTemp(int32_t adcT) {
	int32_t var1, var2, T;
	var1 = ((((adcT>>3) - ((int32_t)dig_T1<<1))) + ((int32_t)dig_T2)) >> 11;
	var2 = (((((adcT>>4) - ((int32_t)dig_T1)) + ((adcT>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t getPressure(int32_t adcP) {
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1)>>1))18;
	var1 = ((((32768 + var1)) * ((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return 0; //avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adcP)-(var2>>12)))*3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)var1);
	}
	else {
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}