#include "SPI.h"
#include "MS5611_SPI.h"
#include "timer.h"	
#include "math.h"
#include "global.h"

unsigned short setup,CRC,C1,C2,C3,C4,C5,C6;
//unsigned long D1,D2; 
long TEMP;
long long OFF,SENS,dT;
long long OFF2,SENS2,T2,AUX,AUX2;
void data2tempeture()
{  	
	dT = (long long)baro.D2 - ((unsigned long)C5 << 8);
  	TEMP = 2000 + ((C6 * dT) >> 23);
	OFF = ((unsigned long)C2 << 16) + ((C4 * dT) >> 7);
	SENS = ((unsigned long)C1 << 15) + ((C3 * dT) >> 8);
	if(TEMP < 2000){
		T2 = (dT * dT) >> 31;
		AUX = (TEMP - 2000) * (TEMP - 2000);
		OFF2 = 5 * AUX >> 1;
		SENS2 = 5 * AUX >> 2;
		if(TEMP < -1500){
			AUX2 = (TEMP + 1500) * (TEMP + 1500);
			OFF2 += 7 * AUX2;
			SENS2 += 11 * AUX2 / 2;
		}
	}
	else{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;	
	baro.temp = TEMP;
}
void data2pressure()
{	
	baro.pressure = ((baro.D1 * SENS >> 21) - OFF) >> 15;
}
float get_altitude()
{
	float tmp_float, altitude;
	float refPressure=baro.refPressure;
	tmp_float = ((float)baro.pressure / refPressure);
	tmp_float = pow(tmp_float, 0.190295);
	altitude = 44330000 * (1.0 - tmp_float);
	return altitude;
}
void MS5611_init()
{
	MS5611_RESET();
	delay_ms(50);
  	C1 = spi_write_1_read_2(CMD_MS5611_PROM_C1, CS_BARO);
	C2 = spi_write_1_read_2(CMD_MS5611_PROM_C2, CS_BARO);
  	C3 = spi_write_1_read_2(CMD_MS5611_PROM_C3, CS_BARO);
  	C4 = spi_write_1_read_2(CMD_MS5611_PROM_C4, CS_BARO);
  	C5 = spi_write_1_read_2(CMD_MS5611_PROM_C5, CS_BARO);
  	C6 = spi_write_1_read_2(CMD_MS5611_PROM_C6, CS_BARO);
  	spi_write_1_read_2(CMD_MS5611_RESET, CS_BARO);
	spi_write_1_read_2(CMD_MS5611_PROM_CRC, CS_BARO);
}
void MS5611_RESET(void)
{		
	spi_read_write(CMD_MS5611_RESET, CS_BARO, IS_LAST);
}

