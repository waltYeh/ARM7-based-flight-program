#include "EEPROM.h"
#include "../Drivers/TWI.h"
#include "../Main/timer.h"
#include "../Main/global.h"
/*
	eeprom_read(_data1, 18, PAGE_1, 0);
	eeprom_read(_data2, 18, PAGE_2, 0);
//	eeprom_write(data1, 18, PAGE_1, 0);
//	eeprom_write(data2, 18, PAGE_2, 0);
	eeprom_read(_data1, 18, PAGE_2, 0);
	eeprom_read(_data2, 18, PAGE_1, 0);		
*/
int eeprom_write(
	void *data, 
	unsigned char len, 
	unsigned short page, 
	unsigned char word)
{
	unsigned char i;
	for(i=0;i<len;i++){
		i2c_eeprom_write_byte(EEPROM_ADDRESS, page+word+i, ((unsigned char *)data+i));
		delay_ms(10);
	}
	return 1;
}

int eeprom_read(
	char *buffer, 
	unsigned char len, 
	unsigned short page, 
	unsigned char word)
{
	char i;
	for(i=0;i<len;i++){
		i2c_eeprom_read_byte(EEPROM_ADDRESS, page+word+i, (buffer+i));
	}
	return 1;
}

