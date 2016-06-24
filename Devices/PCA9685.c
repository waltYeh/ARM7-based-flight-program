#include "PCA9685.h"
#include "../Drivers/TWI.h"
#include "../Main/timer.h"
#include "../Main/global.h"
#include "string.h"


void pca_init(void)
{
	int freq = 100;
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
void start_pca_write(unsigned short duty[4])
{
//2400~4800 --  1720~3440
	int i;
	char pwm_2_write[8];
	unsigned short pca_duty[4];
	for(i = 0; i < 8; i++)
	{
		pca_duty[i] = (int)duty[i]*1720/9600;
	}

	memcpy(pwm_2_write, pca_duty, 8);
	twi_pca_write_start(pwm_2_write);
	//TODO: put this into a fifo, then send via i2c
}
