#ifndef UIO_H
#define UIO_H
#include "at91sam7s256.h"
#define ON 1
#define OFF 0
void pio_init(void);
void led_ctrl(unsigned int channel,unsigned int control);
void beep(unsigned int control);
void usb_pullup(unsigned int control);
int USB_armed(void);
void usb_in_pin(unsigned int control);
__irq void PIO_handler(void);
#endif
