#include "at91sam7s256.h"	
#include "Pins.h"
#include "../Main/timer.h"
#include "../Main/global.h"
#include "PIO.h"	
//#include "../Main/global.h"
void pio_init()
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;	
	*AT91C_PMC_PCER= (1<<AT91C_ID_PIOA);
	*AT91C_PIOA_PER=LED_MASK|BP|USB_IN_O|LED3|USB_PUP;	  	
	*AT91C_PIOA_OER=LED_MASK|BP|USB_IN_O|LED3|USB_PUP;
	usb_in_pin(OFF);
	usb_pullup(OFF);
	pAIC->AIC_SMR[AT91C_ID_PIOA] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 7;
	pAIC->AIC_SVR[AT91C_ID_PIOA] = (unsigned long)PIO_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_PIOA); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_PIOA);
	*AT91C_PIOA_PER |= IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
	*AT91C_PIOA_ODR |= IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
	*AT91C_PIOA_IDR = 0xFFFFFFFF;
	
	*AT91C_PIOA_MDDR |= IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
	*AT91C_PIOA_PPUDR |= IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
	*AT91C_PIOA_ASR &= ~(IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT);
	*AT91C_PIOA_BSR &= ~(IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT);
	*AT91C_PIOA_OWDR |= IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
	*AT91C_PIOA_ISR;
	*AT91C_PIOA_IER = IN0|USB_VBUS|USB_OUT_I|UNLOCK_BUT;
}
void led_ctrl(unsigned int channel,unsigned int control){
	if(channel==LED1){
		if(control==ON){
			*AT91C_PIOA_CODR=LED1;
		}
		else if(control==OFF){
			*AT91C_PIOA_SODR=LED1;
		}
	}
	else if(channel==LED2){
		if(control==ON){
			*AT91C_PIOA_CODR=LED2;
		}
		else if(control==OFF){
			*AT91C_PIOA_SODR=LED2;
		}
	}
	else if(channel==LED3){
		if(control==ON){
			*AT91C_PIOA_CODR=LED3;//reversed
		}
		else if(control==OFF){
			*AT91C_PIOA_SODR=LED3;
		}
	}
}
void beep(unsigned int control)
{
	if(control==ON){
		*AT91C_PIOA_SODR|=BP;
	}
	else if(control==OFF){
		*AT91C_PIOA_CODR|=BP;
	}
}
void usb_in_pin(unsigned int control)
{
	if(control==ON){
		*AT91C_PIOA_SODR|=USB_IN_O;
	}
	else if(control==OFF){
		*AT91C_PIOA_CODR|=USB_IN_O;
	}
}
void usb_pullup(unsigned int control)
{
	if(control==ON){
		*AT91C_PIOA_CODR|=USB_PUP;
	}
	else if(control==OFF){
		*AT91C_PIOA_SODR|=USB_PUP;
	}
}
int USB_armed(void)
{
	if(*AT91C_PIOA_PDSR & USB_VBUS){
		usb_pullup(ON);
		return 1;
	}
	else
		return 0;
}
__irq void PIO_handler(void){
	int timePPM=0;
	int status;
	static unsigned short channel=0;
//	static int beben=0;
	status = *AT91C_PIOA_ISR;
    status &= *AT91C_PIOA_IMR;
	if(status & USB_VBUS){
		if (*AT91C_PIOA_PDSR & USB_VBUS){
			myusb.connect_flag = PLUG_IN;
		//	*AT91C_PIOA_IDR = USB_VBUS;
			usb_pullup(ON);
		}
		else{
			usb_pullup(OFF);
		}
	}
	if(status & USB_OUT_I){
		if (*AT91C_PIOA_PDSR & USB_OUT_I){
			myusb.out_coming = 1;
		}
		else{

		}
	}
	if(status & UNLOCK_BUT){
		static int press_time = 0;
		if (*AT91C_PIOA_PDSR & UNLOCK_BUT){
		//now released
			int press_duration = timer_get() - press_time;
			if(press_duration>1500000){
				mode.locked = !mode.locked;
			}
		} else{
		//now pressed down
			press_time = timer_get();
		}
	}
	if(status & IN0){
		if (*AT91C_PIOA_PDSR & IN0){
			timePPM = ppm_get_time();
			if(timePPM>=0 && timePPM<=4000){
				if(channel < 9){
					cmd.rc[channel]=244*(timePPM-1220)/100;
				}
				channel++;				
			} 
			else{
				channel=0;
			}		
		}
		else{
			ppm_reset_clock();
		}
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_PIOA);
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR;
	return;
}
