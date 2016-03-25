#include "SPI.h"
#include "MPU6000_SPI.h"
void mpu6000_config(void)
{
	unsigned char a1[1] = {1};
	unsigned char a2[1] = {3};
	unsigned char a5[1] = {0};
	unsigned char a6[1] = {0x8};
	
	unsigned char A1 = spi_read_reg(CS_MPU, RA_WHO_AM_I);
	A1 = A1;
	spi_write_reg(CS_MPU, RA_PWR_MGMENT_1, a1[0]);
	spi_write_reg(CS_MPU, RA_SMPLRT_DIV, a2[0]);
	spi_write_reg(CS_MPU, RA_CONFIG, a2[0]);
	spi_write_reg(CS_MPU, RA_GYRO_CONFIG, a5[0]);
	spi_write_reg(CS_MPU, RA_ACCEL_CONFIG, a6[0]);
	spi_write_reg(CS_MPU, RA_USER_CTRL, a5[0]);
	spi_write_reg(CS_MPU, RA_INT_PIN_CFG, a5[0]);
	spi_write_reg(CS_MPU, RA_INT_ENABLE, a5[0]);
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


