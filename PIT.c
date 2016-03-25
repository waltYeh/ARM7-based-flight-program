#include "at91sam7s256.h"
#include "PIT.h"
#include "Pins.h"
#include "global.h"

#if ORIGINAL_FREQ
#define PIV (3000000/PIT_FREQ-1)//599//59
#elif DOUBLED_FREQ
#define PIV (6000000/PIT_FREQ-1)
#endif

__irq void pit_int_handler(void)
{

	*AT91C_AIC_ICCR |= (1 << AT91C_ID_SYS); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR; /* Ack & End of Interrupt */
}

void pit_init () 
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PITC_PIMR = AT91C_PITC_PITIEN | AT91C_PITC_PITEN | (PIV);	
	pAIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_SYS] = (unsigned long) pit_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_SYS); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_SYS); 
	
}
