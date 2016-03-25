#ifndef ATT_MAT_CTRL_C_H
#define ATT_MAT_CTRL_C_H
extern void vector_test(void);
extern void matrix_test(void);
extern void alt_kalman_filter(float process_Q[3],float measure_R[3],float dt,float Resource[3],float x_out[3],char accFlag,char baroFlag,char gpsFlag);
#endif
