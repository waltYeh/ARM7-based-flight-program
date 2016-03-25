#include "at91sam7s256.h"
#include "timer.h"
#include "SPI.h"
#include "global.h"
static int delay_count=0;
static int ppm_ms_clock=0;//in unit of ms
static int timer_ms=0;

void ppm_clock_init(){
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER |=(1<<AT91C_ID_TC0);
	pAIC->AIC_SMR[AT91C_ID_TC0] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 5;
	pAIC->AIC_SVR[AT91C_ID_TC0] = (unsigned long) ppm_ms_clock_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TC0); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TC0);
	*AT91C_TC0_CMR= AT91C_TC_WAVE |
    				AT91C_TC_WAVESEL_UP_AUTO|
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_AEEVT_NONE|
                    AT91C_TC_ASWTRG_NONE|
                    AT91C_TC_BCPB_NONE|
                    AT91C_TC_BCPC_NONE |
                    AT91C_TC_BEEVT_NONE|
                    AT91C_TC_BSWTRG_NONE |
                    AT91C_TC_CLKS_TIMER_DIV1_CLOCK |								
                    AT91C_TC_EEVT_TIOB;								
	*AT91C_TC0_IER=AT91C_TC_CPCS;
	#if ORIGINAL_FREQ
	*AT91C_TC0_RC=23962;//T=(2/MCK)*RC, where T=0.001s, MCK=18.432e6*(26/5)/2=47923200Hz
	#elif DOUBLED_FREQ
	*AT91C_TC0_RC=47924;
	#endif
	
	*AT91C_TC0_CCR |=(0x0<<1);
	*AT91C_TC0_CCR |=AT91C_TC_CLKEN;
	*AT91C_TC0_CCR |=AT91C_TC_SWTRG; 
}

__irq void ppm_ms_clock_int_handler(void){
   	int status0;
	status0=*AT91C_TC0_SR;
	status0=status0;
	ppm_ms_clock++;
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TC0);
	*AT91C_AIC_EOICR=1;	
}

long ppm_get_time(void)//in unit of 1us
{
	#if ORIGINAL_FREQ
	return (ppm_ms_clock*1000+(*AT91C_TC0_CV)/24);
	#elif DOUBLED_FREQ
	return (ppm_ms_clock*1000+(*AT91C_TC0_CV)/48);
	#endif
	
}
void ppm_reset_clock(void)
{
	ppm_ms_clock=0;
	*AT91C_TC0_CCR |=AT91C_TC_SWTRG;
}
void timer_init(){
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER |=(1<<AT91C_ID_TC2);
	pAIC->AIC_SMR[AT91C_ID_TC2] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 4;
	pAIC->AIC_SVR[AT91C_ID_TC2] = (unsigned long) timer_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TC2); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TC2);
	*AT91C_TC2_CMR= AT91C_TC_WAVE |
    				AT91C_TC_WAVESEL_UP_AUTO|
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_AEEVT_NONE|
                    AT91C_TC_ASWTRG_NONE|
                    AT91C_TC_BCPB_NONE|
                    AT91C_TC_BCPC_NONE |
                    AT91C_TC_BEEVT_NONE|
                    AT91C_TC_BSWTRG_NONE |
                    AT91C_TC_CLKS_TIMER_DIV1_CLOCK |								
                    AT91C_TC_EEVT_TIOB;								
	*AT91C_TC2_IER=AT91C_TC_CPCS;
	#if ORIGINAL_FREQ
	*AT91C_TC2_RC=23962;//T=(2/MCK)*RC, where T=0.001s, MCK=18.432e6*(26/5)/2=47923200Hz
	#elif DOUBLED_FREQ
	*AT91C_TC2_RC=47924;
	#endif
	
	*AT91C_TC2_CCR |=(0x0<<1);
	*AT91C_TC2_CCR |=AT91C_TC_CLKEN;
	*AT91C_TC2_CCR |=AT91C_TC_SWTRG; 
}
void delay_ms(int i)//i is in unit of ms
{	 
	 delay_count=i;
	 while(!(delay_count==0));
}
__irq void timer_int_handler(void){
   	int status0;
	static unsigned short sensor_cnt = 0;
	status0=*AT91C_TC2_SR;
	status0=status0;
	timer_ms++;
	if(myusb.rcv_timeout>0){
		myusb.rcv_timeout--;
	}
	if(delay_count != 0)
		delay_count--;
	if(smpl.sens_rdy){
		if(sensor_cnt%2 == 0){//even
			spi_dma_refill(GYRO_ACC);
		}
		else{//odd
			if(sensor_cnt == 1){//trig
				if(baro.temp_pres_switch == PRES_SWITCH){
					spi_dma_refill(GYRO_PTRIG);	
				}
				else{
					spi_dma_refill(GYRO_TTRIG);
				}
			}
			else if(sensor_cnt == 13){//read
				spi_dma_refill(GYRO_TPGET);
			}
			else{
				spi_dma_refill(GYRO);
			}
		}
		if(sensor_cnt==15)
			sensor_cnt = 0;
		else
			sensor_cnt++;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TC2);
	*AT91C_AIC_EOICR=1;	
}

long timer_get(void)//in unit of 1us
{
	#if ORIGINAL_FREQ
	return (timer_ms*1000+(*AT91C_TC2_CV)/24);
	#elif DOUBLED_FREQ
	return (timer_ms*1000+(*AT91C_TC2_CV)/48);
	#endif
}
void timer_reset(void)
{
	timer_ms=0;
	*AT91C_TC2_CCR |=AT91C_TC_SWTRG;
}
