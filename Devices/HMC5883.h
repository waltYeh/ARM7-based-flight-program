#ifndef HMC5883_H
#define HMC5883_H

#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'

#define HMC5983_ADDRESS 		0x1E
#define HMC5983_WRITE			0x3C
#define HMC5983_READ 			0x3D

#define ADDR_CONF_A				0x00
#define ADDR_CONF_B				0x01
#define ADDR_MODE				0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08
#define ADDR_STATUS				0x09
#define HMC5983_ID_A			0x0A
#define HMC5983_ID_B			0x0B
#define HMC5983_ID_C			0x0C

/* modes not changeable outside of driver */
#define HMC5883L_TURN_ON		(0 << 7) 

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define NONE_OUTPUT_RATE			(0x7 << 2)

#define HMC5883L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5883L_GAIN_DEFAULT		(1 << 5)  /*earth mag 0.5~0.6 Gauss*/

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

void hmc5883_config(void);
int hmc5883_read(char *buffer);


#endif
