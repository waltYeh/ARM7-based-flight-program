#include "at91sam7s256.h"
#include "PWM.h"
#include "Pins.h"
#include "../Main/global.h"
void pwm_configure_channel (unsigned int channel, unsigned int prescaler,unsigned int alignment, unsigned int polarity){
    if ((AT91C_BASE_PWMC->PWMC_SR & (1 << channel)) != 0) {   
        AT91C_BASE_PWMC->PWMC_DIS = 1 << channel;   
        while ((AT91C_BASE_PWMC->PWMC_SR & (1 << channel)) != 0);   
    }      
    AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CMR = prescaler | alignment | (polarity<<9);
	AT91C_BASE_PWMC->PWMC_IDR = 0x0000000F; 	
}

void pwm_configure_clock(unsigned int PWM_MR_VAL){
	*AT91C_PWMC_MR = PWM_MR_VAL;
}

void pwm_set_period(unsigned int channel, unsigned short period){
    if ((AT91C_BASE_PWMC->PWMC_SR & (1 << channel)) == 0) {   
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CPRDR = period;   
    }     
    else {   
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CMR |= AT91C_PWMC_CPD;   
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CUPDR = period;   
    }
}

void pwm_set_duty_cycle(unsigned int channel, unsigned short duty){  
    if ((AT91C_BASE_PWMC->PWMC_SR & (1 << channel)) == 0) {   
  
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CDTYR = duty;   
    }   
    else {   
   
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CMR &= ~AT91C_PWMC_CPD;   
        AT91C_BASE_PWMC->PWMC_CH[channel].PWMC_CUPDR = duty;   
    }
}

void pwm_disable_channel(unsigned int channel){
	AT91C_BASE_PWMC->PWMC_DIS = 1<< channel;
}

void pwm_enable_channel(unsigned int channel){
	AT91C_BASE_PWMC->PWMC_ENA = 1<< channel;
}


void pwm_init(void){
	*AT91C_PMC_PCER |= (1<<AT91C_ID_PWMC);
	*AT91C_PIOA_PDR |= PWM_MASK;
	*AT91C_PIOA_BSR |= PWM_MASK;
	#if ORIGINAL_FREQ
	pwm_configure_clock(0x00000014);
	#elif DOUBLED_FREQ
	pwm_configure_clock(0x0000060F);
	#endif
	pwm_configure_channel(0, AT91C_PWMC_CPRE_MCKA, 0, 1);
	pwm_configure_channel(1, AT91C_PWMC_CPRE_MCKA, 0, 1);
	pwm_configure_channel(2, AT91C_PWMC_CPRE_MCKA, 0, 1);	
	pwm_configure_channel(3, AT91C_PWMC_CPRE_MCKA, 0, 1);
	pwm_set_period(0, 6000);
	pwm_set_period(1, 6000);
	pwm_set_period(2, 6000);
	pwm_set_period(3, 6000);
	pwm_set_duty_cycle(0, 0);
	pwm_set_duty_cycle(1, 0);
	pwm_set_duty_cycle(2, 0);
	pwm_set_duty_cycle(3, 0);	 //here must set 0 first, or the polarity will be wrong
	pwm_enable_channel(0);
	pwm_enable_channel(1);
	pwm_enable_channel(2);
	pwm_enable_channel(3);
	pwm_set_duty_cycle(0, 2400);
	pwm_set_duty_cycle(1, 2400);
	pwm_set_duty_cycle(2, 2400);
	pwm_set_duty_cycle(3, 2400);
}
void motor_cut(void)
{
	pwm_set_duty_cycle(0, 2400);
	pwm_set_duty_cycle(1, 2400);
	pwm_set_duty_cycle(2, 2400);
	pwm_set_duty_cycle(3, 2400);
}
