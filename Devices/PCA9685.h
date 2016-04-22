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

#define PWM2_ON_L 0x0E
#define PWM2_ON_H 0x0F
#define PWM2_OFF_L 0x10
#define PWM2_OFF_H 0x11

#define PWM3_ON_L 0x12
#define PWM3_ON_H 0x13
#define PWM3_OFF_L 0x14
#define PWM3_OFF_H 0x15

#define PWM4_ON_L 0x16
#define PWM4_ON_H 0x17
#define PWM4_OFF_L 0x18
#define PWM4_OFF_H 0x19

#define PWM5_ON_L 0x1A
#define PWM5_ON_H 0x1B
#define PWM5_OFF_L 0x1C
#define PWM5_OFF_H 0x1D

#define PWM6_ON_L 0x1E
#define PWM6_ON_H 0x1F
#define PWM6_OFF_L 0x20
#define PWM6_OFF_H 0x21

#define PWM7_ON_L 0x22
#define PWM7_ON_H 0x23
#define PWM7_OFF_L 0x24
#define PWM7_OFF_H 0x25


#define PRE_SCALE_REG 0xFE
#endif
