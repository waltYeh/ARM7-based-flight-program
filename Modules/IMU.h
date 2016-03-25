#ifndef IMU_H
#define IMU_H

extern void gyro_calibration(void);

void data_conclude(char switcher);
void continue_cps_read(void);
void acc_lowpass_biascorr(short ax, short ay, short az);
void gyro_lowpass_biascorr(short gx, short gy, short gz, unsigned char store);

extern void imu_IIR_init(void);
#endif
