#ifndef PCA_H
#define PCA_H
void pca_init(void);
void pca_write(void);
#define PAC9685_ADDRESS 0x40
#define MODE_1_REG 0x00
#define MODE_2_REG 0x01
#define PWM0_ON_L 0x06
#define PWM0_ON_H 0x07
#define PWM0_OFF_L 0x08
#define PWM0_OFF_H 0x09

#define PWM1_ON_L 0x0A
#define PWM1_ON_H 0x0B
#define PWM1_OFF_L 0x0C
#define PWM1_OFF_H 0x0D
#endif
