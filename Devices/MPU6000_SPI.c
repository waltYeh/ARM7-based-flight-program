#include "../Drivers/SPI.h"
#include "MPU6000_SPI.h"
void mpu6000_config(void)
{
	unsigned char A1 = spi_read_reg(CS_MPU, RA_WHO_AM_I);
	A1 = A1;
	spi_write_reg(CS_MPU, RA_PWR_MGMENT_1, 0x1);
	spi_write_reg(CS_MPU, RA_SMPLRT_DIV, 0x0);
	spi_write_reg(CS_MPU, RA_CONFIG, 0x3);//42Hz lps, same in px4
	spi_write_reg(CS_MPU, RA_GYRO_CONFIG, 0x0);//+-250deg/s
	spi_write_reg(CS_MPU, RA_ACCEL_CONFIG, 0x8);//+-4g
	spi_write_reg(CS_MPU, RA_USER_CTRL, 0x0);
	spi_write_reg(CS_MPU, RA_INT_PIN_CFG, 0x0);
	spi_write_reg(CS_MPU, RA_INT_ENABLE, 0x0);
}
void mpu6000_read_all(char *buffer)
{
	int i;
	for (i=0; i<6; i++){
		*(buffer + i) = spi_read_reg(CS_MPU, (0x3B+i));
	}
	for (i=0; i<6; i++){
		*(buffer + 6 + i) = spi_read_reg(CS_MPU, (0x43+i));
	}
}


