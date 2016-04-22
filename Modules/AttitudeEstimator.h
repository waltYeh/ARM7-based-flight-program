#ifndef ATTITUDE_H
#define ATTITUDE_H
void rate_IIR_init(void);
#if OLD_ATT
void marg_update(void);
void attitude_compute(void);
#elif NEW_ATT
void attitude_compute(void);
void MARG_update(void);
void AR_predict(void);
#elif MADGWICK_ATT
void marg_update(void);
void attitude_compute(void);
#elif ORIGINAL_MAD
void marg_update(void);
void attitude_compute(void);
#endif
float data_2_angle(float x, float y, float z);
void quarternion_init(void);
float inv_sqrt(float x);
void body2glob(int body[], int glob[], short dimension);
void glob2body(int body[], int glob[], short dimension);//body=inv(R)*glob
#endif
