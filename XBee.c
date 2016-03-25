#include "at91sam7s256.h"
#include "XBee.h"
#include "string.h"
#include "global.h"
#define VICON_BUFFER_SIZE 14
char xbee_buffer0[VICON_BUFFER_SIZE];
char xbee_buffer1[VICON_BUFFER_SIZE];
unsigned int vicon_buf_ptr=0;
//stores the array offset of the last received byte
unsigned int l_buf_ptr_vicon=0;
unsigned char buf_ptr_buffer_vicon=0;
unsigned char l_buf_ptr_buffer_vicon=0;
unsigned int package_len_vicon=0;
unsigned char buffer_change_times_vicon=0;
#if INDOOR
char tx_buffer[29];
#elif OUTDOOR
char tx_buffer[21];
#endif
unsigned char xbee_buffer_num=0;
void xbee_init(void)
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_US0);		
	*AT91C_PIOA_PDR|=0x00000060;		
	*AT91C_PIOA_ASR|=0x00000060;		
	*AT91C_US0_CR=0XAC;				
	*AT91C_US0_MR=0X8C0;
	#if INDOOR
	*AT91C_US0_RTOR=20;	
	#endif
	*AT91C_US0_IER=0x00;
	*AT91C_US0_IDR=0xFFFF;			
	*AT91C_US0_IER|=AT91C_US_ENDRX;
	*AT91C_US0_IER|=AT91C_US_ENDTX;	
	#if INDOOR
	*AT91C_US0_IER |= AT91C_US_TIMEOUT;
	#endif
	#if ORIGINAL_FREQ			
		*AT91C_US0_BRGR=52;//26;//312;//52;	57600 Baud 52,115200 26 
	#elif DOUBLED_FREQ
		*AT91C_US0_BRGR=104;
	#endif
	*AT91C_US0_RPR=(unsigned int)xbee_buffer1;
	*AT91C_US0_RCR=VICON_BUFFER_SIZE;
	*AT91C_US0_RNPR=(unsigned int)xbee_buffer0;
	*AT91C_US0_RNCR=VICON_BUFFER_SIZE;	
	*AT91C_US0_TPR=(unsigned int)tx_buffer;
	#if OUTDOOR
		*AT91C_US0_TCR=21;
	#elif INDOOR
		*AT91C_US0_TCR=29;
	#endif
	*AT91C_US0_TNPR=(unsigned int)0;
	*AT91C_US0_TNCR=0;				
	pAIC->AIC_SMR[AT91C_ID_US0] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_US0] = (unsigned long) usart_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_US0); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_US0); 
	*AT91C_US0_CR=0X50;
	*AT91C_US0_PTCR|=AT91C_PDC_RXTEN;	
}
#if INDOOR
void refill_tx(void *data,unsigned short length)
{
	unsigned char protocol_head[3]={'>','*','>'};
	unsigned char protocol_last[3]={'<','#','<'};
	unsigned char packetdescriptor='c';
	short crc_result = crc16(data,length);
	memcpy(tx_buffer,protocol_head,3);
	memcpy(tx_buffer+3,&length,2);
	memcpy(tx_buffer+5,&packetdescriptor, 1);
	memcpy(tx_buffer+6,data,18);
	memcpy(tx_buffer+24,&crc_result,2);
	memcpy(tx_buffer+26,protocol_last,3);
//	if(smpl.tx_finished){
		*AT91C_US0_PTCR|=AT91C_PDC_TXTEN;
		smpl.tx_finished=0;
//	}
}
#elif OUTDOOR
void refill_tx(void *data,unsigned short length)
{
	tx_buffer[0]='h';
	memcpy(tx_buffer+1,data,18);
	tx_buffer[19]='\r';
	tx_buffer[20]='\n';
	if(smpl.tx_finished){
		*AT91C_US0_PTCR|=AT91C_PDC_TXTEN;
		smpl.tx_finished=0;
	}
}
#endif
#if INDOOR
__irq void usart_int_handler(void){
	if(*AT91C_US0_CSR & AT91C_US_ENDRX){
		buffer_change_times_vicon++;
		if(xbee_buffer_num==0){
			xbee_buffer_num=1;
			*AT91C_US0_RNPR=(unsigned int)xbee_buffer1;
			*AT91C_US0_RNCR=VICON_BUFFER_SIZE;
		}
		else{
			xbee_buffer_num=0;
			*AT91C_US0_RNPR=(unsigned int)xbee_buffer0;
			*AT91C_US0_RNCR=VICON_BUFFER_SIZE;
		}
	}
	if(*AT91C_US0_CSR & AT91C_US_ENDTX){
		smpl.tx_finished = 1;
		*AT91C_US0_TPR=(unsigned int)tx_buffer;
		*AT91C_US0_TCR=29;
		*AT91C_US0_PTCR|=AT91C_PDC_TXTDIS;
	}
	if(*AT91C_US0_CSR & AT91C_US_TIMEOUT){
		*AT91C_US0_CR|=AT91C_US_STTTO;
		l_buf_ptr_vicon = vicon_buf_ptr;
		l_buf_ptr_buffer_vicon = buf_ptr_buffer_vicon;
		vicon_buf_ptr = VICON_BUFFER_SIZE - *AT91C_US0_RCR - 1;
		buf_ptr_buffer_vicon = xbee_buffer_num;
		package_len_vicon = vicon_buf_ptr + buffer_change_times_vicon * VICON_BUFFER_SIZE - l_buf_ptr_vicon;		
		buffer_change_times_vicon = 0;
		if(package_len_vicon == VICON_BUFFER_SIZE)
			vicon.xbeeflag = 1;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US0); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = 0x1; // Ack & End of Interrupt
}
unsigned char get_xbee_data(void)
{	
	short x,y,z,zero;	
	unsigned char bt;
	int j;
	unsigned char descriptor = 0;
	unsigned short crc_sent;
	unsigned short crc_result;
	unsigned char data_store[VICON_BUFFER_SIZE];
	unsigned int reading_ptr = l_buf_ptr_vicon;
	unsigned char reading_buf = l_buf_ptr_buffer_vicon;	
	unsigned char data_not_lost = 0;
	for(j=0;j<VICON_BUFFER_SIZE;j++){
		reading_ptr++;
		if(reading_ptr > VICON_BUFFER_SIZE - 1){
			reading_ptr = 0;
			reading_buf = (!reading_buf) & 1;//either 0 or 1
		}
		if(reading_buf == 0){
			bt=xbee_buffer0[reading_ptr]; 
		}
		else{
			bt=xbee_buffer1[reading_ptr]; 
		}
		data_store[j] = bt;
	}
	if(data_store[0] == '>' && data_store[1] == '*' &&data_store[2] == '>')	{
		descriptor = data_store[3];
	}
	if (descriptor == 0x63){
		crc_result = crc16(data_store+4,8);
		memcpy(&crc_sent, data_store + 12, 2);
	}
	if (crc_result == crc_sent){
		memcpy(&x,data_store+4,2);
		memcpy(&y,data_store+6,2);
		memcpy(&z,data_store+8,2);
		memcpy(&zero,data_store+10,2);
		zero = zero;
		if(x == 0 && y == 0 && z == 0){
			data_not_lost = 0;
		}else{
			vicon.x = x;
			vicon.y = y;
			vicon.z = z;
			data_not_lost = 1;
		}		
	}
	return data_not_lost;
}	
#elif OUTDOOR
__irq void usart_int_handler(void){	
	int status;
	if(*AT91C_US0_CSR & AT91C_US_ENDTX){//new this is a endtx interrupt
		smpl.tx_finished = 1;
		*AT91C_US0_TPR=(unsigned int)tx_buffer;
		*AT91C_US0_TCR=21;
		*AT91C_US0_PTCR|=AT91C_PDC_TXTDIS;		
	}
	if(*AT91C_US0_CSR & AT91C_US_RXRDY)//new this is a receive interrupt
	{
	;
	}
	status = *AT91C_US0_CSR;
    status &= *AT91C_US0_IMR;
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US0); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = 0x1; // Ack & End of Interrupt
}
#endif
unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;
    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}
unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;
    for (i=0;i<cnt;i++){
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;
}
void uart_send_data(unsigned char *buffer,unsigned char length){	//write continuously
	while (!((*AT91C_US0_CSR) & AT91C_US_TXRDY));
	while(length--){
		uart_write_char(*(buffer++));
	}	
}
void uart_write_char(unsigned char ch){		//most basic
	while (!((*AT91C_US0_CSR) & AT91C_US_TXRDY));   
    *AT91C_US0_THR = ch;
}
