#ifndef ADC_H
#define ADC_H
void adc_init(void);
void adc_start_conversion(void);
unsigned short adc_get_converted(void);
#endif

