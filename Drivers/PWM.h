#ifndef PWM_H
#define PWM_H

extern void pwm_configure_channel (unsigned int channel, unsigned int prescaler,unsigned int alignment, unsigned int polarity);
extern void pwm_configure_clock(unsigned int PWM_MR_VAL);
extern void pwm_set_period(unsigned int channel, unsigned short period);
extern void pwm_set_duty_cycle(unsigned int channel, unsigned short dutycycle);
extern void pwm_disable_channel(unsigned int channel);
extern void pwm_enable_channel(unsigned int channel);
extern void pwm_init(void);
void motor_cut(void);
#endif
