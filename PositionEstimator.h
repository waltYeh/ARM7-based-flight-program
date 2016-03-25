#ifndef FILTER_H
#define FILTER_H
//float alt_kalman_filter(int Z_k[4],float Q[3],float R[4],short dt,int x_out[3]);
void inertial_filter_predict(short dt, int x[2], int acc);
void inertial_filter_correct(int e, short dt, int x[2], char i, float w);
#if OUTDOOR
void gps_pos_init(void);
void gps2xyz(short dt);
void gps_pos_corr(short dt);
#elif INDOOR
void vicon_pos_init(void);
void vicon_pos_corr(short dt);
#endif
void zero_pos_corr(short dt);
void corr_geo_acc(int avgGlobAcc[3]);
void get_geo_acc(int globAcc[3]);
void pos_predict(short dt,unsigned int record_count);
void baro_pos_corr(short dt);
void sonar_pos_corr(short dt);

#endif
