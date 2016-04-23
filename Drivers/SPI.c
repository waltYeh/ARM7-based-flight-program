#include "SPI.h"
#include "../Main/global.h"
#include "at91sam7s256.h"
#include "../Modules/IMU.h"
#include "PIO.h"
const unsigned int spibuf_ga_tx[15] = //enough for only giving the first reg address
	{0x000B00BB,0x000B00BC,0x000B00BD,0x000B00BE,0x000B00BF,0x000B00C0,
	0x000B00C1,0x000B00C2,
	0x000B00C3,0x000B00C4,0x000B00C5,0x000B00C6,0x000B00C7,0x000B00C8,
	0x010B0000};
unsigned int spibuf_ga_rx[15];
unsigned int spibuf_gb_tx[19] = {
	0x000B00BB,0x000B00BC,0x000B00BD,0x000B00BE,0x000B00BF,0x000B00C0,
	0x000B00C1,0x000B00C2,
	0x000B00C3,0x000B00C4,0x000B00C5,0x000B00C6,0x000B00C7,0x000B00C8,
	0x010B0000,
	0,0,0,0};
unsigned int spibuf_gb_rx[19];
unsigned char spi_status=0;
unsigned char baro2get=0;
unsigned char spi_fast_ready = 0;
void spi_init(void)
{	   
	#define SPI_BAUDRATE 1000
	#define MCK 48000
	#define CS_SPCK_DELAY 3
	#define DLYBCS  ((unsigned int) 0x0A << 24)
	int baudrate_div = MCK / SPI_BAUDRATE;
	int dlybs = CS_SPCK_DELAY * MCK / SPI_BAUDRATE / 2;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_SPI);
	*AT91C_PIOA_ASR|=AT91C_PA11_NPCS0|AT91C_PA12_MISO
		|AT91C_PA13_MOSI|AT91C_PA14_SPCK;
	*AT91C_PIOA_BSR|= AT91C_PA30_NPCS2;
	*AT91C_PIOA_PDR|=AT91C_PA11_NPCS0|AT91C_PA12_MISO
		|AT91C_PA13_MOSI|AT91C_PA14_SPCK|AT91C_PA30_NPCS2;	
	*AT91C_PIOA_PPUER = AT91C_PA12_MISO|AT91C_PA13_MOSI|AT91C_PA14_SPCK;
	*AT91C_PIOA_OER|= AT91C_PA30_NPCS2|AT91C_PA11_NPCS0;
	*AT91C_SPI_CR = AT91C_SPI_SPIEN | AT91C_SPI_SWRST;
	*AT91C_SPI_CR = AT91C_SPI_SPIEN;
	*AT91C_SPI_MR=AT91C_SPI_MSTR|AT91C_SPI_PS_VARIABLE|AT91C_SPI_MODFDIS|DLYBCS|AT91C_SPI_PCS;
	//DLYBCS by default sets 6 MCK
	
	AT91C_SPI_CSR[CS_BARO]|=(0x0<<24)|(dlybs<<16)|(baudrate_div << 8)
		|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;
	//mode 0
	AT91C_SPI_CSR[CS_MPU]|=(0x0<<24)|(dlybs<<16)|(baudrate_div << 8)
		|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;

	*AT91C_SPI_IDR|=0x03FF;//all disabled	

	*AT91C_AIC_IDCR = (1<<AT91C_ID_SPI);
}
unsigned char spi_read_write(unsigned char order, unsigned char cs, unsigned int last_tx)
{
	unsigned int pcs = 0x0F;
	pcs &= ~(1<<cs);
	while (!(*AT91C_SPI_SR & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=(last_tx << 24)|(pcs<<16)|order; 
	while (!(*AT91C_SPI_SR & AT91C_SPI_RDRF));
	return(*AT91C_SPI_RDR);
}
unsigned short spi_write_1_read_2(unsigned char data_w, unsigned char cs)
{
	unsigned char byteH,byteL;
  	unsigned short return_value;
	spi_read_write(data_w, cs, NOT_LAST);
	byteH = spi_read_write(0, cs, NOT_LAST);
	byteL = spi_read_write(0, cs, IS_LAST);
	return_value = (((unsigned short)byteH)<<8) | (byteL);
	return(return_value);    
}
unsigned char spi_read_reg(unsigned char cs, unsigned char reg)
{
	unsigned char ret;
	spi_read_write((1<<7)|reg, cs, NOT_LAST);
	ret = spi_read_write(0, cs, IS_LAST);
	return ret;
}
unsigned char spi_write_reg(unsigned char cs, unsigned char reg, unsigned char data)
{
	unsigned char status;
	spi_read_write((0<<7)|reg, cs, NOT_LAST);
	status = spi_read_write(data, cs, IS_LAST);
	return status;
}


void spi_fast_init(void)
{
	int baudrate_div = 3;
	int dlybs = 3;
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	pAIC->AIC_SMR[AT91C_ID_SPI] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_SPI] = (unsigned long)spi_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_SPI); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_SPI);
	//only after reset can we reprogram the baudrate
	*AT91C_SPI_CR = AT91C_SPI_SWRST;
	*AT91C_SPI_CR = AT91C_SPI_SPIEN;
	*AT91C_SPI_MR=AT91C_SPI_MSTR|AT91C_SPI_PS_VARIABLE|AT91C_SPI_MODFDIS|DLYBCS|AT91C_SPI_PCS;
	AT91C_SPI_CSR[CS_BARO]|=(0x0<<24)|(dlybs<<16)|(baudrate_div << 8)
		|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;
	//mode 0
	AT91C_SPI_CSR[CS_MPU]|=(0x0<<24)|(dlybs<<16)|(baudrate_div << 8)
		|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;
	*AT91C_SPI_IDR = 0x03FF;
	*AT91C_SPI_IER = AT91C_SPI_OVRES;
	spi_fast_ready = 1;
}
__irq void spi_int_handler(void)
{
	unsigned int status = *AT91C_SPI_SR;
	if(status & AT91C_SPI_RXBUFF){
		*AT91C_SPI_PTCR|=AT91C_PDC_TXTDIS|AT91C_PDC_RXTDIS;
		*AT91C_SPI_IDR|=AT91C_SPI_RXBUFF;
		spi_dma_decode(spi_status);
		spi_status = SPI_IDLE;		
	}
	if(status & AT91C_SPI_OVRES){

	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_SPI);
	*AT91C_AIC_EOICR = 0x1;
}


void spi_dma_refill(unsigned char switcher)
{
	unsigned short i;
	spi_status = switcher;
	*AT91C_SPI_PTCR|=AT91C_PDC_TXTDIS|AT91C_PDC_RXTDIS;
	switch(spi_status){
	case GYRO_ACC:
		*AT91C_SPI_RPR=(unsigned int)spibuf_ga_rx;
		*AT91C_SPI_RCR=15;
		*AT91C_SPI_TPR=(unsigned int)spibuf_ga_tx;
		*AT91C_SPI_TCR=15;
		break;
	case GYRO:
		*AT91C_SPI_RPR=(unsigned int)spibuf_gb_rx;
		*AT91C_SPI_RCR=15;
		*AT91C_SPI_TPR=(unsigned int)spibuf_gb_tx;
		*AT91C_SPI_TCR=15;
		break;
	case GYRO_PTRIG:
		spibuf_gb_tx[15] = 0x010E0048;//D1
		*AT91C_SPI_RPR=(unsigned int)spibuf_gb_rx;
		*AT91C_SPI_RCR=16;
		*AT91C_SPI_TPR=(unsigned int)spibuf_gb_tx;
		*AT91C_SPI_TCR=16;
		baro2get = P2GET;		
		break;
	case GYRO_TTRIG:
		spibuf_gb_tx[15] = 0x010E0058;//D2
		*AT91C_SPI_RPR=(unsigned int)spibuf_gb_rx;
		*AT91C_SPI_RCR=16;
		*AT91C_SPI_TPR=(unsigned int)spibuf_gb_tx;
		*AT91C_SPI_TCR=16;
		baro2get = T2GET;
		break;
	case GYRO_TPGET:
		for(i=15;i<18;i++){
			spibuf_gb_tx[i] = 0x000E0000;
		}
		spibuf_gb_tx[18] = 0x010E0000;
		*AT91C_SPI_RPR=(unsigned int)spibuf_gb_rx;
		*AT91C_SPI_RCR=19;
		*AT91C_SPI_TPR=(unsigned int)spibuf_gb_tx;
		*AT91C_SPI_TCR=19;
		break;
	default:
		break;
	}//end of switch
	if(spi_fast_ready){
		*AT91C_SPI_PTCR|=AT91C_PDC_TXTEN|AT91C_PDC_RXTEN;
		*AT91C_SPI_IER|=AT91C_SPI_RXBUFF;
	}
}
void spi_dma_decode(unsigned char switcher)
{
	unsigned char xH, xL, yH, yL, zH, zL;
	switch(switcher){
	case GYRO_ACC:
		xH = spibuf_ga_rx[9];
		xL = spibuf_ga_rx[10];
		yH = spibuf_ga_rx[11];
		yL = spibuf_ga_rx[12];
		zH = spibuf_ga_rx[13];
		zL = spibuf_ga_rx[14];
		gyro_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 0);
		xH = spibuf_ga_rx[1];
		xL = spibuf_ga_rx[2];
		yH = spibuf_ga_rx[3];
		yL = spibuf_ga_rx[4];
		zH = spibuf_ga_rx[5];
		zL = spibuf_ga_rx[6];
		acc_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 0);
		smpl.Flag500Hz = 1;
		break;
	case GYRO:
		xH = spibuf_gb_rx[9];
		xL = spibuf_gb_rx[10];
		yH = spibuf_gb_rx[11];
		yL = spibuf_gb_rx[12];
		zH = spibuf_gb_rx[13];
		zL = spibuf_gb_rx[14];
		gyro_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		xH = spibuf_ga_rx[1];
		xL = spibuf_ga_rx[2];
		yH = spibuf_ga_rx[3];
		yL = spibuf_ga_rx[4];
		zH = spibuf_ga_rx[5];
		zL = spibuf_ga_rx[6];
		acc_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		break;
	case GYRO_PTRIG:
		xH = spibuf_gb_rx[9];
		xL = spibuf_gb_rx[10];
		yH = spibuf_gb_rx[11];
		yL = spibuf_gb_rx[12];
		zH = spibuf_gb_rx[13];
		zL = spibuf_gb_rx[14];
		gyro_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		xH = spibuf_ga_rx[1];
		xL = spibuf_ga_rx[2];
		yH = spibuf_ga_rx[3];
		yL = spibuf_ga_rx[4];
		zH = spibuf_ga_rx[5];
		zL = spibuf_ga_rx[6];
		acc_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		break;
	case GYRO_TPGET:
	{
		xH = spibuf_gb_rx[9];
		xL = spibuf_gb_rx[10];
		yH = spibuf_gb_rx[11];
		yL = spibuf_gb_rx[12];
		zH = spibuf_gb_rx[13];
		zL = spibuf_gb_rx[14];
		gyro_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		xH = spibuf_ga_rx[1];
		xL = spibuf_ga_rx[2];
		yH = spibuf_ga_rx[3];
		yL = spibuf_ga_rx[4];
		zH = spibuf_ga_rx[5];
		zL = spibuf_ga_rx[6];
		acc_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		if(baro2get==P2GET){
			unsigned char PH,PM,PL;
			PH = spibuf_gb_rx[16];
			PM = spibuf_gb_rx[17];
			PL = spibuf_gb_rx[18];
			baro.D1=(((unsigned long)PH)<<16) | (((unsigned long)PM)<<8) | ((unsigned long)PL);
			baro.updated = D1_UPDATED;
		}
		else if(baro2get==T2GET){
			unsigned char TH,TM,TL;
			TH = spibuf_gb_rx[16];
			TM = spibuf_gb_rx[17];
			TL = spibuf_gb_rx[18];
				baro.D2=(((unsigned long)TH)<<16) | (((unsigned long)TM)<<8) | ((unsigned long)TL);
				baro.updated = D2_UPDATED;
		}
		baro2get = 0;
		break;
	}
	case GYRO_TTRIG:
		xH = spibuf_gb_rx[9];
		xL = spibuf_gb_rx[10];
		yH = spibuf_gb_rx[11];
		yL = spibuf_gb_rx[12];
		zH = spibuf_gb_rx[13];
		zL = spibuf_gb_rx[14];
		gyro_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		xH = spibuf_ga_rx[1];
		xL = spibuf_ga_rx[2];
		yH = spibuf_ga_rx[3];
		yL = spibuf_ga_rx[4];
		zH = spibuf_ga_rx[5];
		zL = spibuf_ga_rx[6];
		acc_lowpass_biascorr(((short)xH<<8)|(short)xL, ((short)yH<<8)|(short)yL, ((short)zH<<8)|(short)zL, 1);
		break;
	
	default:
		break;
	}
}
