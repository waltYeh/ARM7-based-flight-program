#include "HMC5883.h"
#include "../Drivers/TWI.h"
#include "../Main/timer.h"
#include "../Main/global.h"
void hmc5883_config(void)
{
//	char tmp[1];
	unsigned char a1 = 0x18;//HMC5883L_TURN_ON | HMC5883L_AVERAGING_1 | NONE_OUTPUT_RATE | HMC5883L_MODE_NORMAL;
	unsigned char a2 = 0x20;
	unsigned char a3 = 0x00;//HMC5883L_TURN_ON | MODE_REG_CONTINOUS_MODE;
	i2cwrite(HMC5983_ADDRESS,ADDR_CONF_A,1,&a1);
	delay_ms(10);
	i2cwrite(HMC5983_ADDRESS,ADDR_CONF_B,1,&a2);
	delay_ms(10);
	i2cwrite(HMC5983_ADDRESS,ADDR_MODE,1,&a3);
	delay_ms(10);
}

int hmc5883_read(char *buffer)
{
//	char data[1];
	char tmp[1];
	short i;
//	data[0]=0x01;
	i2cread(HMC5983_ADDRESS,ADDR_STATUS,1,tmp);
	if(tmp[0] & 0x1 == 1){
		 for(i=0;i<6;i++)
		 	i2cread(HMC5983_ADDRESS,(ADDR_DATA_OUT_X_MSB+i),1,&buffer[i]);
		 return 1;
	}
	return 0;
}
