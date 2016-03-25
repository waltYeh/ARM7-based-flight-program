#ifndef TIMER_H
#define TIMER_H

#include "at91sam7s256.h"
void ppm_clock_init(void);
void ppm_reset_clock(void);
long ppm_get_time(void);//in unit of 0.01ms, use TC0
__irq void ppm_ms_clock_int_handler(void);

void delayer_init(void);
void delay_ms(int i);
__irq void delayer_int_handler(void);

void timer_init(void);
void timer_reset(void);
long timer_get(void);//in unit of 0.01ms, use TC0
__irq void timer_int_handler(void);

#endif
