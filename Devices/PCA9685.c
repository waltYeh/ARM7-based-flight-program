#include "PCA9685.h"
#include "../Drivers/TWI.h"
#include "../Main/timer.h"
#include "../Main/global.h"
void pca_init(void)
{
	int freq = 400;
	int prescale = 25000000 / (4096 * freq) - 1;
	unsigned char presc = prescale;
	// Get settings and calc bytes for the different states.
	char settings = 0x00;
	unsigned char sleep	= 0x10;//settings | 0x10;									// Set sleep bit to 1
	unsigned char wake 	= 0x00;//settings & 0xEF;									// Set sleep bit to 0
	unsigned char restart = 0x80;
	unsigned char output = 0x04;
	i2cwtritebyte(PAC9685_ADDRESS, MODE_2_REG, &output);
	settings = 0x00;
	sleep = settings;
	sleep = (sleep & 0x7F)|0x10;
	wake = settings;
	wake = (wake & 0x7F)&0xEF;
	restart = wake | 0x80;
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &sleep);
	i2cwtritebyte(PAC9685_ADDRESS, PRE_SCALE_REG, &presc);
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &wake);
	delay_ms(5);
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &restart);
}
void pca_write(void)
{
	short pca_duty = 1720;
	unsigned char duty_l = pca_duty;
	unsigned char duty_h = pca_duty>>8;
	i2cwtritebyte(PAC9685_ADDRESS, PWM0_OFF_L, &duty_l);
	i2cwtritebyte(PAC9685_ADDRESS, PWM0_OFF_H, &duty_h);
}
void pca_duty(unsigned int channel, unsigned short duty)
{
//2400~4800 --  1720~3440
	unsigned short pca_duty = (int)duty*1720/2400;
    pca_duty = pca_duty;
	//TODO: put this into a fifo, then send via i2c
}
