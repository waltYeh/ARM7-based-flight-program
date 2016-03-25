#ifndef _twi_h_
#define _twi_h_

#define AT91C_EEPROM_READ_OK			0 
#define AT91C_EEPROM_WRITE_OK			0
#define ERROR_TWI 1
void twi_init(void);
unsigned char i2cwtritebyte(unsigned char address, unsigned short reg, unsigned char *data);
unsigned char i2cwrite(unsigned char address, unsigned short reg, unsigned char len, unsigned char *data);
unsigned char i2creadbyte(unsigned char address, unsigned short reg, char *buf);
unsigned char i2cread(unsigned char address, unsigned short reg, unsigned char len, char *buf);
unsigned char i2c_eeprom_write_byte(unsigned char address, unsigned short reg, unsigned char *data);
unsigned char i2c_eeprom_read_byte(unsigned char address, unsigned short reg, char *buf);
typedef struct {
	char *Buf;
	int intAdd[6];
	int devAdd;
	int readCnt;
//	int sizeRemain;
}TWI_Reader ;
void twi_fast_init(void);
int twi_cps_read_start(char *buf);

int twi_gyro_read_start(char *buf);
int twi_acc_read_start(char *buf);


__irq void twi_int_handler(void);
#define ACC_SWITCH 0
#define GYRO_SWITCH 1
#define CPS_SWITCH 2
#define NON_WORKING 3
#define READ_START_NOT_RDY 1
#endif
