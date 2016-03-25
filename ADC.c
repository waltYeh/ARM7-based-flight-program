#include "ADC.h"
#include "global.h"
#include "at91sam7s256.h"
void adc_init(void)
{
    #define adcClock 5000000
	#define mckClock 48000000
    #define startupTime 10
    #define sampleAndHoldTime 1200
	unsigned int prescal;
    unsigned int startup;
    unsigned int shtim;
	//AT91S_ADC *pAdc;
	prescal = (mckClock / (2*adcClock)) - 1;
    startup = ((adcClock/1000000) * startupTime / 8) - 1;
    shtim = (((adcClock/1000000) * sampleAndHoldTime)/1000) - 1;
	*AT91C_PMC_PCER |= (1 << AT91C_ID_ADC);
	*AT91C_ADC_CR = AT91C_ADC_SWRST;
	*AT91C_ADC_MR = AT91C_ADC_TRGEN_DIS 
	| AT91C_ADC_LOWRES_10_BIT 
	| AT91C_ADC_SLEEP_NORMAL_MODE
	| (prescal<<8) //& AT91C_ADC_PRESCAL)
	| (startup<<16)// & AT91C_ADC_STARTUP) 
	| (shtim<<24);// & AT91C_ADC_SHTIM);	
	*AT91C_ADC_CHER = (1 << 7);
}

void adc_start_conversion(void)
{
	*AT91C_ADC_CR = AT91C_ADC_START;
}

unsigned short adc_get_converted(void)
{
//3.75V~818
//4.14V~904
	unsigned short data=0,voltage=0;
	data = *AT91C_ADC_CDR7;
	voltage = constrain(data*195/43,3000,4300);
	return voltage;
}

