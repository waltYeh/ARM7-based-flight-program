#include "PCA9685.h"
#include "../Drivers/TWI.h"
#include "../Main/timer.h"
#include "../Main/global.h"
void pca_init(void)
{
	int freq = 500;

	// To set pwm frequency we have to set the prescale register. The formula is:
	// prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
	// Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
	int prescale = 25000000 / (4096 * freq) - 1;
	unsigned char presc = prescale;
	// Get settings and calc bytes for the different states.
	char settings = 0;//wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
	unsigned char sleep	= 0x10;//settings | 0x10;									// Set sleep bit to 1
	unsigned char wake 	= 0x00;//settings & 0xEF;									// Set sleep bit to 0
	unsigned char restart = 0x80;										// Set restart bit to 1
	i2creadbyte(PAC9685_ADDRESS, MODE_1_REG, &settings);
//	sleep = settings;
//	sleep |= (1<<4);
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &sleep);
	delay_ms(2);
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &presc);
	i2creadbyte(PAC9685_ADDRESS, MODE_1_REG, &settings);
//	wake = settings;
//	wake &= ~(1<<4);
//	restart = wake | 0x80;
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &wake);
	delay_ms(2);
	i2cwtritebyte(PAC9685_ADDRESS, MODE_1_REG, &restart);
	delay_ms(2);
//	sleep = settings | 
	// Go to sleep, set prescale and wake up again.
//	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
//	wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
//	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

	// Now wait a millisecond until oscillator finished stabilizing and restart PWM.
//	delay(1);
//	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart)
}
void pca_write(void)
{
	short duty = 1000;
	unsigned char duty_l = duty;
	unsigned char duty_h = duty>>8;
	i2cwtritebyte(PAC9685_ADDRESS, PWM1_OFF_L, &duty_l);
	i2cwtritebyte(PAC9685_ADDRESS, PWM1_OFF_H, &duty_h);
}
