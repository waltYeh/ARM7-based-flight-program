#include "at91sam7s256.h"
#include "TWI.h"
#include "PIO.h"
#include "../Modules/IMU.h"
#include "../Main/timer.h"
#include "../Main/global.h"
#include "../Devices/PCA9685.h"
#define CPS_READER 0
#define PCA_WRITER 0
TWI_Reader readers[1]={
//acc
//	{0,{0x3B,0x3C,0x3D,0x3E,0x3F,0x40},0x68,0},
//gyro
//	{0,{0x43,0x44,0x45,0x46,0x47,0x48},0x68,0},
//compass
	{0,{0x03,0x04,0x05,0x06,0x07,0x08},0x1E,0}
};	

TWI_Writer pca_writer = {0,{0x08,0x09,0x0C,0x0D,0x10,0x11,0x14,0x15},0x40,0};

char stop_signal=0;
//set at stop_twi_after_complete(),cleared at last compass interrupt
char read_rdy_flag=1;
//set at stop_twi_now() and last compass interrupt,
//cleared at twi_g_a_read_start() and twi_cps_read_start()
char working_reader=NON_WORKING;
//set as G_A_SWITCH at twi_g_a_read_start(),
//set as CPS_SWITCH at twi_cps_read_start(),
//set as NON_WORKING at stop_twi_now() and last compass interrupt
char working_writer=0;
#define PCA_WRITING 1
#define NON_WRITING 0
char compass_prepare_flag = 0; // COMPASS 读取标志位决定是否读取罗盘

void compass_read_prepared()   // COMPASS 读取标志位置1
{
 compass_prepare_flag = 1; 
}

void twi_init(void)
{
// 	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_TWI);		//TWI时钟使能
	*AT91C_PIOA_PDR|=0x18;		
	*AT91C_PIOA_ASR|=0x18;		//分配TWI
//	*AT91C_PIOA_PDR|=0x18;
//	*AT91C_PIOA_PPUER|=0x18;
	*AT91C_PIOA_MDER|=0x18;		//pull-up
	*AT91C_TWI_IDR = 0x107;		
	*AT91C_TWI_CR =  AT91C_TWI_SWRST;
	*AT91C_TWI_CR =  AT91C_TWI_MSEN;
	*AT91C_TWI_CWGR = 0x11920;
	*AT91C_AIC_IDCR = (1<<AT91C_ID_TWI);		
}

unsigned char i2cwtritebyte(unsigned char address, unsigned short reg, unsigned char *data)
{
	*AT91C_TWI_CR =  AT91C_TWI_MSEN; 
    *AT91C_TWI_MMR = ( (address<<16) | AT91C_TWI_IADRSZ_1_BYTE ) & ~AT91C_TWI_MREAD;            
    *AT91C_TWI_IADR = reg;             
    *AT91C_TWI_THR = *data;       
   	while (!(*AT91C_TWI_SR & AT91C_TWI_TXRDY));   
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));              
    return AT91C_EEPROM_WRITE_OK;
}

unsigned char i2cwrite(unsigned char address, unsigned short reg, unsigned char len, unsigned char *data)
{
	int i;
	int k;
	for(i=0;i<len;i++){
		k=i2cwtritebyte(address,reg,(data+i));	
	}
	return k;	
}

unsigned char i2creadbyte(unsigned char address, unsigned short reg, char *buf)
{
	*AT91C_TWI_CR = 0x00000004;
    *AT91C_TWI_MMR = (address<<16) | AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;       
    *AT91C_TWI_IADR = reg;   
    *AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;   
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));            
    *buf = *AT91C_TWI_RHR;              
    return AT91C_EEPROM_READ_OK; 
}


unsigned char i2cread(unsigned char address, unsigned short reg, unsigned char len, char *buf)
{
	int i;
	int k;
	for(i=0;i<len;i++){
		k=i2creadbyte(address,reg,(buf+i));	
	}
	return k;	
}
unsigned char i2c_eeprom_write_byte(unsigned char address, unsigned short reg, unsigned char *data)
{
	*AT91C_TWI_CR =  AT91C_TWI_MSEN; 
    *AT91C_TWI_MMR = ( (address<<16) | AT91C_TWI_IADRSZ_2_BYTE ) & ~AT91C_TWI_MREAD;            
    *AT91C_TWI_IADR = reg;             
    *AT91C_TWI_THR = *data;       
   	while (!(*AT91C_TWI_SR & AT91C_TWI_TXRDY));
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));  
    return AT91C_EEPROM_WRITE_OK;
}
unsigned char i2c_eeprom_read_byte(unsigned char address, unsigned short reg, char *buf)
{
	*AT91C_TWI_CR = 0x00000004;
    *AT91C_TWI_MMR = (address<<16) | AT91C_TWI_IADRSZ_2_BYTE | AT91C_TWI_MREAD;       
    *AT91C_TWI_IADR = reg;   
	*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;   
    while (!(*AT91C_TWI_SR & AT91C_TWI_RXRDY));            
    *buf = *AT91C_TWI_RHR;   
	while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP)); 
    return AT91C_EEPROM_READ_OK; 
}
void twi_fast_init(void)
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	pAIC->AIC_SMR[AT91C_ID_TWI] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 5;
	pAIC->AIC_SVR[AT91C_ID_TWI] = (unsigned long)twi_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TWI); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TWI);
	*AT91C_TWI_IDR = 0x107;		
}

int twi_cps_read_start(char *buf)
{
	readers[CPS_READER].Buf = buf;
	readers[CPS_READER].readCnt = 0;	
	*AT91C_TWI_MMR = (readers[CPS_READER].devAdd << 16) 
		| AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;
	*AT91C_TWI_IADR = readers[CPS_READER].intAdd[0];
	#if I2C_CONTI
	*AT91C_TWI_CR = AT91C_TWI_START;
	*AT91C_TWI_IER = AT91C_TWI_RXRDY;
	#else
	*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
	*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
	#endif
	working_reader = CPS_READER;
	read_rdy_flag = 0;
	return 0;

}
int twi_pca_write_start(char *buf)
{
	pca_writer.Buf = buf;
	pca_writer.writeCnt = 0;
	*AT91C_TWI_MMR = ((pca_writer.devAdd<<16)|AT91C_TWI_IADRSZ_1_BYTE) & ~AT91C_TWI_MREAD;
	*AT91C_TWI_IADR = pca_writer.intAdd[0];
	*AT91C_TWI_THR = *(pca_writer.Buf);
	*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
	working_writer = PCA_WRITING;
	return 0;

}
void stop_twi_now()
{
	working_reader = NON_WORKING;
	read_rdy_flag = 1;
	*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
}
void stop_twi_after_complete()
{
	stop_signal = 1;
}

__irq void twi_int_handler(void)
{
	int status;
	status = *AT91C_TWI_IMR;
	status = *AT91C_TWI_SR;
	#if I2C_CONTI
	if(status & AT91C_TWI_RXRDY){
		if(working_reader !=NON_WORKING){
			*((readers[working_reader].Buf)+readers[working_reader].readCnt) = *AT91C_TWI_RHR;
			readers[working_reader].readCnt++;
			if(readers[working_reader].readCnt == 5){//go on reading
				*AT91C_TWI_CR = AT91C_TWI_STOP;
			}
			else if(readers[working_reader].readCnt == 6){
			//reading of a sensor complete, now conclude
				if(working_reader==CPS_READER){//cps over		
					data_conclude(CPS_SWITCH);
					sens.mag_updated = 1;		
					working_reader = NON_WORKING;
					read_rdy_flag = 1;
					*AT91C_TWI_IDR = AT91C_TWI_RXRDY;
					
				}				
			}
			else if(readers[working_reader].readCnt > 6){
			}
		}
		else{
		}
	}
	#else
	if(status & AT91C_TWI_TXCOMP){
		if(working_reader !=NON_WORKING){
			*((readers[working_reader].Buf)+readers[working_reader].readCnt) = *AT91C_TWI_RHR;
			readers[working_reader].readCnt++;
			if(readers[working_reader].readCnt < 6){//go on reading
				*AT91C_TWI_IADR = readers[working_reader].intAdd[readers[working_reader].readCnt];
				*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
			}
			else{//reading of a sensor complete, now conclude
				if(working_reader==CPS_READER){//cps over		
					data_conclude(CPS_SWITCH);
					sens.mag_updated = 1;		
					working_reader = NON_WORKING;
					read_rdy_flag = 1;
					*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
				}				
			}
		}
		else if(working_writer == PCA_WRITING)
		{
		if(pca_writer.writeCnt < 7)//go on reading
			{
				pca_writer.writeCnt++;
				*AT91C_TWI_IADR = pca_writer.intAdd[pca_writer.writeCnt];
				*AT91C_TWI_THR = *(pca_writer.Buf + pca_writer.writeCnt);
			}
			else //reading of a sensor complete, now conclude
				{
					if(compass_prepare_flag==1)	
					{
					compass_prepare_flag=0;
					continue_cps_read();
					}
					else
					{
					working_writer = NON_WRITING;
					*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
					}
				}
		}
		
	}
	
	#endif
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TWI);
	*AT91C_AIC_EOICR = 0x1;
}
