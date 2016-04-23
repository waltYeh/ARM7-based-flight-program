#include "AttitudeEstimator.h"
#include "math.h"
#include "../Main/global.h"
#include "IMU.h"
#include "../Main/timer.h"
#include "../Main/commons.h"
#include "Filter.h"
IIRFilter iir_wx={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_wy={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_wz={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
void rate_IIR_init(float cutoff_freq)
{	
//	float cutoff_freq = 250.0;
	float smpl_freq = 500.0;
	IIR_set_cutoff_freq(&iir_wx, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_wy, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_wz, cutoff_freq, smpl_freq);
	att.rollspeed = IIR_reset(&iir_wx, 0);
	att.pitchspeed = IIR_reset(&iir_wy, 0);
	att.yawspeed = IIR_reset(&iir_wz, 0);

}
#if NEW_ATT
volatile float q0=DSCRT_I,q1=0,q2=0,q3=0;//<<14
#define Kp_ACC 0.2f//0.5f //too much noise if  large, too slow to recover the drift if small
#define Kp_MAG 1.2f//1.2f
#define gyro_bias_weight 0.1f
#define scale_gyr 7506 //correspond to 1 rad/s
float gyr_bias[3]={0,0,0};//change to float??
void marg_update(void) //(1<<14)rad
{
	float norm;
	int m_bdy[3],a_bdy[3],m_glb[3];
	
	#if MAG_PITCH_ROLL
	int ENU_frame[3];
	int bdy_frame[3];
	#else
	int m_err_glb[3], m_err_bdy[3];
	#endif
	int a_R[3];
	int acc_corr[3];
	int mag_corr[3];
	short dt_gyr = 2;//, dt_acc = 2, dt_mag = 2;
	float qa,qb,qc;
	int corr_roll = 0, corr_pitch = 0, corr_yaw = 0;
	int qi0,qi1,qi2,qi3;
	if(1){
		norm = inv_sqrt((int)sens.ax*sens.ax + (int)sens.ay*sens.ay + (int)sens.az*sens.az);       
		a_bdy[0] = ((int)sens.ax<<DSCRT)*norm;
		a_bdy[1] = ((int)sens.ay<<DSCRT)*norm;
		a_bdy[2] = ((int)sens.az<<DSCRT)*norm;
		//	estimated gravity direction (v xyz) is the last rol of R
		a_R[0] = att.R[2][0];
		a_R[1] = att.R[2][1];
		a_R[2] = att.R[2][2];
		// error is obtained by cross production
		//c = a % c
		//rc = (ra % rb)/r
		acc_corr[0] = (a_bdy[1]*a_R[2] - a_bdy[2]*a_R[1])>>DSCRT;
		acc_corr[1] = (a_bdy[2]*a_R[0] - a_bdy[0]*a_R[2])>>DSCRT;
		acc_corr[2] = (a_bdy[0]*a_R[1] - a_bdy[1]*a_R[0])>>DSCRT;
		
		acc_corr[0] *= Kp_ACC;
		acc_corr[1] *= Kp_ACC;
		acc_corr[2] *= Kp_ACC;
		
		corr_roll += acc_corr[0];
		corr_pitch += acc_corr[1];
		corr_yaw += acc_corr[2];
		
	}
	if(sens.mag_updated){
		sens.mag_updated = 0;
		norm = inv_sqrt((int)sens.mx*sens.mx + (int)sens.my*sens.my + (int)sens.mz*sens.mz);
		m_bdy[0]=((int)sens.mx<<DSCRT)*norm;
		m_bdy[1]=((int)sens.my<<DSCRT)*norm;
		m_bdy[2]=((int)sens.mz<<DSCRT)*norm;
		body2glob(m_bdy, m_glb, 3);
		
	#if MAG_PITCH_ROLL
		ENU_frame[0]=0;
		ENU_frame[1]=sqrt(m_glb[0]*m_glb[0] + m_glb[1]*m_glb[1]);
    	ENU_frame[2]=m_glb[2];
		glob2body(bdy_frame, ENU_frame, 3);
		mag_corr[0] =(m_bdy[1]*bdy_frame[2] - m_bdy[2]*bdy_frame[1])>>DSCRT;
		mag_corr[1] =(m_bdy[2]*bdy_frame[0] - m_bdy[0]*bdy_frame[2])>>DSCRT;
		mag_corr[2] =(m_bdy[0]*bdy_frame[1] - m_bdy[1]*bdy_frame[0])>>DSCRT;
	#else
		m_err_glb[2] = atan2(m_glb[0],m_glb[1]) * DSCRT_F;
		m_err_glb[0] = 0;
		m_err_glb[1] = 0;
		glob2body(m_err_bdy, m_err_glb, 3);
		mag_corr[0] = m_err_bdy[0];
		mag_corr[1] = m_err_bdy[1];
		mag_corr[2] = m_err_bdy[2];
	#endif
		mag_corr[0] *= Kp_MAG;
		mag_corr[1] *= Kp_MAG;
		mag_corr[2] *= Kp_MAG;
		
		corr_roll += mag_corr[0];
		corr_pitch += mag_corr[1];
		corr_yaw += mag_corr[2];
	}
	gyr_bias[0] += constrain_f(corr_roll * gyro_bias_weight * dt_gyr,-100.0f,100.0f);
	gyr_bias[1] += constrain_f(corr_pitch * gyro_bias_weight * dt_gyr,-100.0f,100.0f);
	gyr_bias[2] += constrain_f(corr_yaw * gyro_bias_weight * dt_gyr,-100.0f,100.0f);
//	data2[3] = gyr_bias[0];
//	data2[4] = gyr_bias[1];
//	data2[5] = gyr_bias[2];
	
	att.rollspeed=((int)sens.gx<<DSCRT)/scale_gyr + gyr_bias[0]/1000;
	att.pitchspeed=((int)sens.gy<<DSCRT)/scale_gyr + gyr_bias[1]/1000;   
	att.yawspeed=((int)sens.gz<<DSCRT)/scale_gyr + gyr_bias[2]/1000;
	corr_roll += att.rollspeed;
	corr_pitch += att.pitchspeed;
	corr_yaw += att.yawspeed;
	
	// updating quarternion using first order approximation
	//q and g is in (<<14),  halfT in 1e-3
	qa=q0;
	qb=q1;
	qc=q2;
//	corr_roll = corr_roll;
//	corr_pitch = corr_pitch;
//	corr_yaw = corr_yaw;
	q0 += ((-qb*corr_roll - qc*corr_pitch - q3*corr_yaw)/DSCRT_F)*dt_gyr/2000;
	q1 += ((qa*corr_roll + qc*corr_yaw - q3*corr_pitch)/DSCRT_F)*dt_gyr/2000;
	q2 += ((qa*corr_pitch - qb*corr_yaw + q3*corr_roll)/DSCRT_F)*dt_gyr/2000;
	q3 += ((qa*corr_yaw + qb*corr_pitch - qc*corr_roll)/DSCRT_F)*dt_gyr/2000;  
	// quarternion normalization
	norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 =(q0*DSCRT_F)*norm;
	q1 =(q1*DSCRT_F)*norm;
	q2 =(q2*DSCRT_F)*norm;
	q3 =(q3*DSCRT_F)*norm;
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	//obtain the rotation matrix that can be applied elsewhere
	att.R[0][0]=(1<<DSCRT) - ((qi2 * qi2 + qi3 * qi3)>>(DSCRT-1));
	att.R[0][1]=(qi1 * qi2 - qi0 * qi3)>>(DSCRT-1);
	att.R[0][2]=(qi1 * qi3 + qi0 * qi2)>>(DSCRT-1);
	att.R[1][0]=(qi1 * qi2 + qi0 * qi3)>>(DSCRT-1);
	att.R[1][1]=(1<<DSCRT) - ((qi1 * qi1 + qi3 * qi3)>>(DSCRT-1));
	att.R[1][2]=(qi2 * qi3 - qi0 * qi1)>>(DSCRT-1);
	att.R[2][0]=(qi1 * qi3 - qi0 * qi2)>>(DSCRT-1);
	att.R[2][1]=(qi2 * qi3 + qi0 * qi1)>>(DSCRT-1);
	att.R[2][2]=(1<<DSCRT) - ((qi1 * qi1 + qi2 * qi2)>>(DSCRT-1));
	att.q[0] = qi0;
	att.q[1] = qi1;
	att.q[2] = qi2;
	att.q[3] = qi3;
}

void attitude_compute(void)
{
	static short turn=0;
	static int l_yaw=0;
	int pitch,roll,yaw;  //in (1<<14)rad
	marg_update();
	pitch = -asin(att.R[2][0]/DSCRT_F)*DSCRT_F;
	roll  = atan2(att.R[2][1], att.R[2][2])*DSCRT_F;
	yaw = -atan2(att.R[0][1], att.R[1][1])*DSCRT_F;
	//yaw changes continuously without jumping from -180 to 180
	if(yaw < (-(DSCRT_I*19/20)*PI) && l_yaw>((DSCRT_I*19/20)*PI))turn++;//0.95*16384=15565
	if(yaw > ((DSCRT_I*19/20)*PI) && l_yaw<(-(DSCRT_I*19/20)*PI))turn--;	
	l_yaw = yaw;
	yaw += turn*((DSCRT_I*2)*PI);	
	att.pitch=pitch;
	att.roll=roll;
	att.yaw=yaw;
}
#elif OLD_ATT
volatile float q0=DSCRT_I,q1=0,q2=0,q3=0;//<<14
#define Kp_ACC 0.5f //too much noise if  large, too slow to recover the drift if small
#define Ki_ACC 0.0025f//flutuates if too large, slow towards steady state if zero
#define Kp_MAG 2.0f
#define Ki_MAG 0.0025f

void imu_update(void) //(1<<14)rad
{
	//angular parameters should use rad but not raw data
	float norm;
	int m[3],a[3],w[3];
	int v[3];
	int ex, ey, ez;
	int eMx,eMy,eMz;
	int h[3],b[3];
	short dt = 2;
	static int exInt=0,eyInt=0,ezInt=0;
	static int eMxInt=0,eMyInt=0,eMzInt=0;
	float qa,qb,qc;
	int qi0,qi1,qi2,qi3;
	int g_roll = att.rollspeed, g_pitch = att.pitchspeed, g_yaw = att.yawspeed;	       
	norm = inv_sqrt((int)sens.ax*sens.ax + sens.ay*sens.ay + sens.az*sens.az);       
	a[0] = ((int)sens.ax<<DSCRT)*norm;
	a[1] = ((int)sens.ay<<DSCRT)*norm;
	a[2] = ((int)sens.az<<DSCRT)*norm;      	
	norm = inv_sqrt((int)sens.mx*sens.mx + sens.my*sens.my + sens.mz*sens.mz);
	m[0]=((int)sens.mx<<DSCRT)*norm;
	m[1]=((int)sens.my<<DSCRT)*norm;
	m[2]=((int)sens.mz<<DSCRT)*norm;
//  body mag (m xyz) towards geo mag (b xyz)
	body2glob(m, h, 3);	
	b[0]=0;
	b[1]=sqrt(h[0]*h[0] + h[1]*h[1]);
    b[2]=h[2];
//	geo mag (b xyz) towards estimated body mag (w xyz)
	glob2body(w, b, 3);
//	estimated gravity direction (v xyz) is the last rol of R
	v[0] = att.R[2][0];
	v[1] = att.R[2][1];
	v[2] = att.R[2][2];
	// error is obtained by cross production
	//c = a % c
	//rc = (ra % rb)/r
	ex = (a[1]*v[2] - a[2]*v[1])>>DSCRT;
	ey = (a[2]*v[0] - a[0]*v[2])>>DSCRT;
	ez = (a[0]*v[1] - a[1]*v[0])>>DSCRT;
	eMx =(m[1]*w[2] - m[2]*w[1])>>DSCRT;
	eMy =(m[2]*w[0] - m[0]*w[2])>>DSCRT;
	eMz =(m[0]*w[1] - m[1]*w[0])>>DSCRT;   
	if(ex != 0 && ey != 0 && ez != 0){
	// integration term of error
		exInt += ex * dt * Ki_ACC;
		eyInt += ey * dt * Ki_ACC;
		ezInt += ez * dt * Ki_ACC;
		eMxInt += eMx * dt * Ki_MAG;
		eMyInt += eMy * dt * Ki_MAG;
		eMzInt += eMz * dt * Ki_MAG; 
	// corrected gyro data
	//maybe this is the bias free gyro data
	//err is in (<<14), g is in (<<14)
		g_roll += (Kp_ACC*ex + exInt/1000.0f + Kp_MAG*eMx + eMxInt/1000.0f);
		g_pitch += (Kp_ACC*ey + eyInt/1000.0f + Kp_MAG*eMy + eMyInt/1000.0f);
		g_yaw += (Kp_ACC*ez + ezInt/1000.0f + Kp_MAG*eMz + eMzInt/1000.0f);
	}
	// updating quarternion using first order approximation
	//q and g is in (<<14),  halfT in 1e-3
	qa=q0;
	qb=q1;
	qc=q2;
	q0 += ((-qb*g_roll - qc*g_pitch - q3*g_yaw)/DSCRT_F)*dt/2000;
	q1 += ((qa*g_roll + qc*g_yaw - q3*g_pitch)/DSCRT_F)*dt/2000;
	q2 += ((qa*g_pitch - qb*g_yaw + q3*g_roll)/DSCRT_F)*dt/2000;
	q3 += ((qa*g_yaw + qb*g_pitch - qc*g_roll)/DSCRT_F)*dt/2000;  
	// quarternion normalization
	norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 =(q0*DSCRT_F)*norm;
	q1 =(q1*DSCRT_F)*norm;
	q2 =(q2*DSCRT_F)*norm;
	q3 =(q3*DSCRT_F)*norm;
	//obtain the rotation matrix that can be applied elsewhere
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	//obtain the rotation matrix that can be applied elsewhere
	att.R[0][0]=(1<<DSCRT) - ((qi2 * qi2 + qi3 * qi3)>>(DSCRT-1));
	att.R[0][1]=(qi1 * qi2 - qi0 * qi3)>>(DSCRT-1);
	att.R[0][2]=(qi1 * qi3 + qi0 * qi2)>>(DSCRT-1);
	att.R[1][0]=(qi1 * qi2 + qi0 * qi3)>>(DSCRT-1);
	att.R[1][1]=(1<<DSCRT) - ((qi1 * qi1 + qi3 * qi3)>>(DSCRT-1));
	att.R[1][2]=(qi2 * qi3 - qi0 * qi1)>>(DSCRT-1);
	att.R[2][0]=(qi1 * qi3 - qi0 * qi2)>>(DSCRT-1);
	att.R[2][1]=(qi2 * qi3 + qi0 * qi1)>>(DSCRT-1);
	att.R[2][2]=(1<<DSCRT) - ((qi1 * qi1 + qi2 * qi2)>>(DSCRT-1));
	att.q[0] = qi0;
	att.q[1] = qi1;
	att.q[2] = qi2;
	att.q[3] = qi3;
}

void attitude_compute(void)
{
	#define scale_gyr 7506 //correspond to 1 rad/s
	#define scale_acc_gravity 8192//correspond to 9.8 m/s2
	#define beta 199/200
	#define beta_v 19/20
	static int pitch_v=0,roll_v=0,yaw_v=0;   //in m_rad
	static short turn=0;
	static int l_yaw=0;
	int pitch,roll,yaw;  //in (1<<14)rad
	int ax_comp, ay_comp;
//	short dt = 2;
	imu_update();
	pitch = -asin(att.R[2][0]/DSCRT_F)*DSCRT_F;
	roll  = atan2(att.R[2][1], att.R[2][2])*DSCRT_F;
	yaw = -atan2(att.R[0][1], att.R[1][1])*DSCRT_F;
	//yaw changes continuously without jumping from -180 to 180
	if(yaw < (-(DSCRT_I*19/20)*PI) && l_yaw>((DSCRT_I*19/20)*PI))turn++;//0.95*16384=15565
	if(yaw > ((DSCRT_I*19/20)*PI) && l_yaw<(-(DSCRT_I*19/20)*PI))turn--;	
	l_yaw = yaw;
	yaw += turn*((DSCRT_I*2)*PI);
		
	pitch_v = ((int)sens.gx<<DSCRT)/scale_gyr;
	roll_v = ((int)sens.gy<<DSCRT)/scale_gyr;
	//high pass filter for angular velocity
	pitch_v = pitch_v - pitch_v * beta_v + ((int)sens.gy<<DSCRT)/scale_gyr * beta_v;   
	roll_v = roll_v - roll_v * beta_v + ((int)sens.gx<<DSCRT)/scale_gyr * beta_v;
	yaw_v = yaw_v - yaw_v * beta_v + ((int)sens.gz<<DSCRT)/scale_gyr * beta_v;
	//compensation filter for angles
	ax_comp = ((int)sens.ax<<DSCRT)/scale_acc_gravity;
	ay_comp = ((int)sens.ay<<DSCRT)/scale_acc_gravity;
	pitch = pitch * beta + pitch_v * ATT_PERIOD * beta / 1000 - ax_comp + ax_comp * beta;
	roll = roll * beta + roll_v * ATT_PERIOD  * beta / 1000 + ay_comp - ay_comp * beta; 

	att.pitchspeed=pitch_v;
	att.rollspeed=roll_v;
	att.yawspeed=yaw_v;
	att.pitch=pitch;
	att.roll=roll;
	att.yaw=yaw;
	
//	yaw_v = sens.gz*1000/scale_gyr;	
}
#elif MADGWICK_ATT
volatile float q0=DSCRT_I,q1=0,q2=0,q3=0;//<<14
#define beta 0.020f//0.025f//0.06f
#define gyro_bias_weight 0.02f//0.02f
#define scale_gyr 7506 //correspond to 1 rad/s
float gyr_bias[3]={0,0,0};//change to float??
void marg_update(void) //(1<<14)rad
{
	float norm;
	int a_bdy[3];
//	int m_bdy_est[3];
	int mag_corr[3] = {0,0,0};
	#if DSCRT_15
//	static int m_ENU[3]={0,16626,-28169};
	#elif DSCRT_14
//	static int m_ENU[3]={0,8313,-14084};
	#endif
	short dt_gyr = 2;//, dt_acc = 2, dt_mag = 2;
	int corr_x = 0, corr_y = 0, corr_z = 0;
	int qi0,qi1,qi2,qi3;
	int Df[4] = {0,0,0,0};
	int fa[3];//, fm[3]={0,0,0};
	int wx,wy,wz;
	int wq0,wq1,wq2,wq3;
	int i,j;
	int Jcb_a[3][4];
//	int Jcb_m[3][4];
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	if(1){
		norm = inv_sqrt((int)sens.ax*sens.ax + (int)sens.ay*sens.ay + (int)sens.az*sens.az);
		if(isfinite(norm)){     
			a_bdy[0] = ((int)sens.ax<<DSCRT)*norm;
			a_bdy[1] = ((int)sens.ay<<DSCRT)*norm;
			a_bdy[2] = ((int)sens.az<<DSCRT)*norm;
		}
		else{
			a_bdy[0] = att.R[2][0];
			a_bdy[1] = att.R[2][1];
			a_bdy[2] = att.R[2][2];
		}
		for(i=0;i<3;i++)
			fa[i] = att.R[2][i] - a_bdy[i];//obj func
		Jcb_a[0][0] = -qi2<<1;
		Jcb_a[0][1] = qi3<<1;
		Jcb_a[0][2] = -qi0<<1;
		Jcb_a[0][3] = qi1<<1;
		Jcb_a[1][0] = qi1<<1;
		Jcb_a[1][1] = qi0<<1;
		Jcb_a[1][2] = qi3<<1;
		Jcb_a[1][3] = qi2<<1;
		Jcb_a[2][0] = 0;
		Jcb_a[2][1] = -qi1<<2;
		Jcb_a[2][2] = -qi2<<2;
		Jcb_a[2][3] = 0;
		for(i=0;i<4;i++){
			for(j=0;j<3;j++){
				Df[i] += Jcb_a[j][i] * fa[j]>>DSCRT;
			}
		}
	}
	if(1){//sens.mag_updated){
		
/*		norm = inv_sqrt((int)sens.mx*sens.mx + (int)sens.my*sens.my + (int)sens.mz*sens.mz);
		if(isfinite(norm)){
			m_bdy[0]=((int)sens.mx<<DSCRT)*norm;
			m_bdy[1]=((int)sens.my<<DSCRT)*norm;
			m_bdy[2]=((int)sens.mz<<DSCRT)*norm;
		}
		else{
			m_bdy[0]=att.R[1][0];
			m_bdy[1]=att.R[1][1];
			m_bdy[2]=att.R[1][2];
		}
		body2glob(m_bdy, m_glb, 3);
		norm = inv_sqrt(m_glb[0]*m_glb[0] + m_glb[1]*m_glb[1]);
		if(isfinite(norm)){
			m_glb[0]=(m_glb[0]<<DSCRT)*norm;
			m_glb[1]=(m_glb[1]<<DSCRT)*norm;
			m_glb[2]=0;
			
			glob2body(m_bdy_est, m_glb, 3);

			for(i=0;i<3;i++)
				fm[i] = att.R[1][i] - m_bdy_est[i];
		}
		else{
			q0 = q0;
		}
		data2[0] = fm[0];
		data2[1] = fm[1];
		data2[2] = fm[2];
		Jcb_m[0][0] =  qi3<<1;
		Jcb_m[0][1] =  qi2<<1;
		Jcb_m[0][2] =  qi1<<1;
		Jcb_m[0][3] =  qi0<<1;
		Jcb_m[1][0] =  0;
		Jcb_m[1][1] = -qi1<<2;
		Jcb_m[1][2] =  0;
		Jcb_m[1][3] = -qi3<<2;
		Jcb_m[2][0] = -qi1<<1;
		Jcb_m[2][1] = -qi0<<1;
		Jcb_m[2][2] =  qi3<<1;
		Jcb_m[2][3] =  qi2<<1;
		
		for(i=0;i<4;i++){
			for(j=0;j<3;j++){
				Df[i] += (Jcb_m[j][i] * fm[j]>>DSCRT)*8;
			}
		}
*/
		int m_err_glb[3], m_err_bdy[3];
		int m_bdy[3], m_glb[3];
		#define Kp_MAG 2.0f
		
		sens.mag_updated = 0;
		norm = inv_sqrt((int)sens.mx*sens.mx + (int)sens.my*sens.my + (int)sens.mz*sens.mz);
		m_bdy[0]=((int)sens.mx<<DSCRT)*norm;
		m_bdy[1]=((int)sens.my<<DSCRT)*norm;
		m_bdy[2]=((int)sens.mz<<DSCRT)*norm;
		body2glob(m_bdy, m_glb, 3);
		

		m_err_glb[2] = atan2(m_glb[0],m_glb[1]) * DSCRT_F;
		m_err_glb[0] = 0;
		m_err_glb[1] = 0;
		glob2body(m_err_bdy, m_err_glb, 3);
		mag_corr[0] = m_err_bdy[0];
		mag_corr[1] = m_err_bdy[1];
		mag_corr[2] = m_err_bdy[2];

		mag_corr[0] *= Kp_MAG;
		mag_corr[1] *= Kp_MAG;
		mag_corr[2] *= Kp_MAG;
//		corr_x += mag_corr[0];
//		corr_y += mag_corr[1];
		corr_z += mag_corr[2];
	//	gyr_bias[0] += constrain_f(mag_corr[0] * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
	//	gyr_bias[1] += constrain_f(mag_corr[1] * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
		gyr_bias[2] -= constrain_f(corr_z * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
		
//		data2[0] = mag_corr[0];
//		data2[1] = mag_corr[1];
//		data2[2] = mag_corr[2];	
	}
	
	norm = inv_sqrt((float)Df[0] * Df[0] + (float)Df[1] * Df[1] + (float)Df[2] * Df[2] + (float)Df[3] * Df[3]);
	if(isfinite(norm)){
		Df[0] =(Df[0]<<DSCRT)*norm;
		Df[1] =(Df[1]<<DSCRT)*norm;
		Df[2] =(Df[2]<<DSCRT)*norm;
		Df[3] =(Df[3]<<DSCRT)*norm;
	}
	else{
		Df[0] =0;
		Df[1] =0;
		Df[2] =0;
		Df[3] =0;
		
	}
	corr_x += (qi0*Df[1] - qi1*Df[0] - qi2*Df[3] + qi3*Df[2])>>(DSCRT-1);
	corr_y += (qi0*Df[2] + qi1*Df[3] - qi2*Df[0] - qi3*Df[1])>>(DSCRT-1);
//	corr_z += (qi0*Df[3] - qi1*Df[2] + qi2*Df[1] - qi3*Df[0])>>(DSCRT-1);
	
	
	
	gyr_bias[0] += constrain_f(corr_x * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
	gyr_bias[1] += constrain_f(corr_y * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
//	gyr_bias[2] += constrain_f(corr_z * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);

/*	
	att.rollspeed=((int)sens.gx<<DSCRT)/scale_gyr - gyr_bias[0];
	att.pitchspeed=((int)sens.gy<<DSCRT)/scale_gyr - gyr_bias[1];   
	att.yawspeed=((int)sens.gz<<DSCRT)/scale_gyr - gyr_bias[2];
	#define ZOOM 2
	wx = (att.rollspeed + mag_corr[0])>>ZOOM;
	wy = (att.pitchspeed + mag_corr[1])>>ZOOM;
	wz = (att.yawspeed + mag_corr[2])>>ZOOM;


	att.rate[0] = IIR_apply(&iir_wx, att.rollspeed);
	att.rate[1] = IIR_apply(&iir_wx, att.pitchspeed);
	att.rate[2] = IIR_apply(&iir_wx, att.yawspeed);
*/
	wx = ((int)sens.gx<<(DSCRT-1))/scale_gyr - gyr_bias[0];
	wy = ((int)sens.gy<<(DSCRT-1))/scale_gyr - gyr_bias[1];
	wz = ((int)sens.gz<<(DSCRT-1))/scale_gyr - gyr_bias[2];
//	att.rollspeed = IIR_apply(&iir_wx, wx)*2;
//	att.pitchspeed = IIR_apply(&iir_wy, wy)*2;
//	att.yawspeed = IIR_apply(&iir_wz, wz)*2;
	att.rollspeed = wx*2;
	att.pitchspeed = wy*2;
	att.yawspeed = wz*2;
	
//	#define ZOOM 2
	wx = (wx + mag_corr[0])>>1;
	wy = (wy + mag_corr[1])>>1;
	wz = (wz + mag_corr[2])>>1;
	
	
	wq0 = (-qi1*wx - qi2*wy - qi3*wz)>>(DSCRT-1);
	wq1 = (qi0*wx + qi2*wz - qi3*wy)>>(DSCRT-1);
	wq2 = (qi0*wy - qi1*wz + qi3*wx)>>(DSCRT-1);
	wq3 = (qi0*wz + qi1*wy - qi2*wx)>>(DSCRT-1);

	q0 += (wq0 - beta*Df[0])*dt_gyr/1000.0f;
	q1 += (wq1 - beta*Df[1])*dt_gyr/1000.0f;
	q2 += (wq2 - beta*Df[2])*dt_gyr/1000.0f;
	q3 += (wq3 - beta*Df[3])*dt_gyr/1000.0f;
	// quarternion normalization
	norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if(isfinite(norm)){
		q0 =(q0*DSCRT_F)*norm;
		q1 =(q1*DSCRT_F)*norm;
		q2 =(q2*DSCRT_F)*norm;
		q3 =(q3*DSCRT_F)*norm;
	}
	else{
		q0 = q0;
	}
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	//obtain the rotation matrix that can be applied elsewhere
	att.R[0][0]=(1<<DSCRT) - ((qi2 * qi2 + qi3 * qi3)>>(DSCRT-1));
	att.R[0][1]=(qi1 * qi2 - qi0 * qi3)>>(DSCRT-1);
	att.R[0][2]=(qi1 * qi3 + qi0 * qi2)>>(DSCRT-1);
	att.R[1][0]=(qi1 * qi2 + qi0 * qi3)>>(DSCRT-1);
	att.R[1][1]=(1<<DSCRT) - ((qi1 * qi1 + qi3 * qi3)>>(DSCRT-1));
	att.R[1][2]=(qi2 * qi3 - qi0 * qi1)>>(DSCRT-1);
	att.R[2][0]=(qi1 * qi3 - qi0 * qi2)>>(DSCRT-1);
	att.R[2][1]=(qi2 * qi3 + qi0 * qi1)>>(DSCRT-1);
	att.R[2][2]=(1<<DSCRT) - ((qi1 * qi1 + qi2 * qi2)>>(DSCRT-1));
	att.q[0] = qi0;
	att.q[1] = qi1;
	att.q[2] = qi2;
	att.q[3] = qi3;
//	if(sens.mag_updated){
//		sens.mag_updated = 0;
//		body2glob(m_bdy, m_glb, 3);
//		m_ENU[0]=0;
//		m_ENU[1]=sqrt(m_glb[0]*m_glb[0] + m_glb[1]*m_glb[1]);
//	    m_ENU[2]=m_glb[2];
//	}
}

void attitude_compute(void)
{
	static short turn=0;
	static int l_yaw=0;
	int pitch,roll,yaw;  //in (1<<14)rad
	marg_update();
	pitch = -asin(att.R[2][0]/DSCRT_F)*DSCRT_F;
	roll  = atan2(att.R[2][1], att.R[2][2])*DSCRT_F;
	yaw = -atan2(att.R[0][1], att.R[1][1])*DSCRT_F;
	//yaw changes continuously without jumping from -180 to 180
	if(yaw < (-(DSCRT_I*19/20)*PI) && l_yaw>((DSCRT_I*19/20)*PI))turn++;//0.95*16384=15565
	if(yaw > ((DSCRT_I*19/20)*PI) && l_yaw<(-(DSCRT_I*19/20)*PI))turn--;	
	l_yaw = yaw;
	yaw += turn*((DSCRT_I*2)*PI);	
	att.pitch=pitch;
	att.roll=roll;
	att.yaw=yaw;
}

#elif ORIGINAL_MAD
volatile float q0=DSCRT_I,q1=0,q2=0,q3=0;//<<14
#define beta 0.020f//0.025f//0.06f
#define gyro_bias_weight 0.001f
#define scale_gyr 7506 //correspond to 1 rad/s
float gyr_bias[3]={0,0,0};//change to float??
void marg_update(void) //(1<<14)rad
{
	float norm;
	int m_bdy[3],a_bdy[3],m_glb[3];
	int m_bdy_est[3];
	#if DSCRT_15
	static int m_ENU[3]={0,16626,-28169};
	#elif DSCRT_14
	static int m_ENU[3]={0,8313,-14084};
	#endif
	short dt_gyr = 2;//, dt_acc = 2, dt_mag = 2;
	int corr_x = 0, corr_y = 0, corr_z = 0;
	int qi0,qi1,qi2,qi3;
	int Df[4] = {0,0,0,0};
	int fa[3], fm[3]={0,0,0};
	int wx,wy,wz;
	int wq0,wq1,wq2,wq3;
	int i,j;
	int Jcb_a[3][4];
	int Jcb_m[3][4];
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	if(1){
		norm = inv_sqrt((int)sens.ax*sens.ax + (int)sens.ay*sens.ay + (int)sens.az*sens.az);
		if(isfinite(norm)){     
			a_bdy[0] = ((int)sens.ax<<DSCRT)*norm;
			a_bdy[1] = ((int)sens.ay<<DSCRT)*norm;
			a_bdy[2] = ((int)sens.az<<DSCRT)*norm;
		}
		else{
			a_bdy[0] = att.R[2][0];
			a_bdy[1] = att.R[2][1];
			a_bdy[2] = att.R[2][2];
		}
		for(i=0;i<3;i++)
			fa[i] = att.R[2][i] - a_bdy[i];//obj func
		Jcb_a[0][0] = -qi2<<1;
		Jcb_a[0][1] = qi3<<1;
		Jcb_a[0][2] = -qi0<<1;
		Jcb_a[0][3] = qi1<<1;
		Jcb_a[1][0] = qi1<<1;
		Jcb_a[1][1] = qi0<<1;
		Jcb_a[1][2] = qi3<<1;
		Jcb_a[1][3] = qi2<<1;
		Jcb_a[2][0] = 0;
		Jcb_a[2][1] = -qi1<<2;
		Jcb_a[2][2] = -qi2<<2;
		Jcb_a[2][3] = 0;
		for(i=0;i<4;i++){
			for(j=0;j<3;j++){
				Df[i] += Jcb_a[j][i] * fa[j]>>DSCRT;
			}
		}
	}
	if(sens.mag_updated){
		
//		sens.mag_updated = 0;
		glob2body(m_bdy_est, m_ENU, 3);
		norm = inv_sqrt((int)sens.mx*sens.mx + (int)sens.my*sens.my + (int)sens.mz*sens.mz);
		if(isfinite(norm)){
			m_bdy[0]=((int)sens.mx<<DSCRT)*norm;
			m_bdy[1]=((int)sens.my<<DSCRT)*norm;
			m_bdy[2]=((int)sens.mz<<DSCRT)*norm;
		}
		else{
			m_bdy[0]=m_bdy_est[0];
			m_bdy[1]=m_bdy_est[1];
			m_bdy[2]=m_bdy_est[2];
		}
		
		for(i=0;i<3;i++)
			fm[i] = m_bdy_est[i] - m_bdy[i];

		Jcb_m[0][0] =  (qi3*m_ENU[1] >> (DSCRT-1)) - (qi2*m_ENU[2] >> (DSCRT-1));
		Jcb_m[0][1] =  (qi2*m_ENU[1] >> (DSCRT-1)) + (qi3*m_ENU[2] >> (DSCRT-1));
		Jcb_m[0][2] =  (qi1*m_ENU[1] >> (DSCRT-1)) - (qi0*m_ENU[2] >> (DSCRT-1));
		Jcb_m[0][3] =  (qi0*m_ENU[1] >> (DSCRT-1)) + (qi1*m_ENU[2] >> (DSCRT-1));
		Jcb_m[1][0] =  (qi1*m_ENU[2] >> (DSCRT-1));
		Jcb_m[1][1] = -(qi1*m_ENU[1] >> (DSCRT-2)) + (qi0*m_ENU[2] >> (DSCRT-1));
		Jcb_m[1][2] =  (qi3*m_ENU[2] >> (DSCRT-1));
		Jcb_m[1][3] = -(qi3*m_ENU[1] >> (DSCRT-2)) + (qi2*m_ENU[2] >> (DSCRT-1));
		Jcb_m[2][0] = -(qi1*m_ENU[1] >> (DSCRT-1));
		Jcb_m[2][1] = -(qi0*m_ENU[1] >> (DSCRT-1)) - (qi1*m_ENU[2] >> (DSCRT-2));
		Jcb_m[2][2] =  (qi3*m_ENU[1] >> (DSCRT-1)) - (qi2*m_ENU[2] >> (DSCRT-2));
		Jcb_m[2][3] =   qi2*m_ENU[1] >> (DSCRT-1);
		
		data2[0] = fa[0];
		data2[1] = fa[1];
		data2[2] = fa[2];
		
		for(i=0;i<4;i++){
			for(j=0;j<3;j++){
			//	Df[i] += (Jcb_m[j][i] * fm[j]>>DSCRT)*10;
			}
		}
	}
	
	norm = inv_sqrt(Df[0] * Df[0] + Df[1] * Df[1] + Df[2] * Df[2] + Df[3] * Df[3]);
	if(isfinite(norm)){
		Df[0] =(Df[0]<<DSCRT)*norm;
		Df[1] =(Df[1]<<DSCRT)*norm;
		Df[2] =(Df[2]<<DSCRT)*norm;
		Df[3] =(Df[3]<<DSCRT)*norm;
	}
	else{
		Df[0] =0;
		Df[1] =0;
		Df[2] =0;
		Df[3] =0;
		
	}
	corr_x = (qi0*Df[1] - qi1*Df[0] - qi2*Df[3] + qi3*Df[2])>>(DSCRT-1);
	corr_y = (qi0*Df[2] + qi1*Df[3] - qi2*Df[0] - qi3*Df[1])>>(DSCRT-1);
	corr_z = (qi0*Df[3] - qi1*Df[2] + qi2*Df[1] - qi3*Df[0])>>(DSCRT-1);

	gyr_bias[0] += constrain_f(corr_x * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
	gyr_bias[1] += constrain_f(corr_y * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);
	gyr_bias[2] += constrain_f(corr_z * gyro_bias_weight * dt_gyr/1000.0f,-100.0f,100.0f);

	wx = ((int)sens.gx<<DSCRT)/scale_gyr - gyr_bias[0];
	wy = ((int)sens.gy<<DSCRT)/scale_gyr - gyr_bias[1];   
	wz = ((int)sens.gz<<DSCRT)/scale_gyr - gyr_bias[2];

//	att.rollspeed=wx;
//	att.pitchspeed=wy;   
//	att.yawspeed=wz;
	att.rollspeed  = IIR_apply(&iir_wx, wx);//used for control loop
	att.pitchspeed = IIR_apply(&iir_wy, wy);
	att.yawspeed   = IIR_apply(&iir_wz, wz);
	
	#define ZOOM 2
	wx = wx>>ZOOM;
	wy = wy>>ZOOM;
	wz = wz>>ZOOM;

	wq0 = (-qi1*wx - qi2*wy - qi3*wz)>>(DSCRT-ZOOM+1);
	wq1 = (qi0*wx + qi2*wz - qi3*wy)>>(DSCRT-ZOOM+1);
	wq2 = (qi0*wy - qi1*wz + qi3*wx)>>(DSCRT-ZOOM+1);
	wq3 = (qi0*wz + qi1*wy - qi2*wx)>>(DSCRT-ZOOM+1);

	q0 += (wq0 - beta*Df[0])*dt_gyr/1000.0f;
	q1 += (wq1 - beta*Df[1])*dt_gyr/1000.0f;
	q2 += (wq2 - beta*Df[2])*dt_gyr/1000.0f;
	q3 += (wq3 - beta*Df[3])*dt_gyr/1000.0f;
	// quarternion normalization
	norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if(isfinite(norm)){
		q0 =(q0*DSCRT_F)*norm;
		q1 =(q1*DSCRT_F)*norm;
		q2 =(q2*DSCRT_F)*norm;
		q3 =(q3*DSCRT_F)*norm;
	}
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	//obtain the rotation matrix that can be applied elsewhere
	att.R[0][0]=(1<<DSCRT) - ((qi2 * qi2 + qi3 * qi3)>>(DSCRT-1));
	att.R[0][1]=(qi1 * qi2 - qi0 * qi3)>>(DSCRT-1);
	att.R[0][2]=(qi1 * qi3 + qi0 * qi2)>>(DSCRT-1);
	att.R[1][0]=(qi1 * qi2 + qi0 * qi3)>>(DSCRT-1);
	att.R[1][1]=(1<<DSCRT) - ((qi1 * qi1 + qi3 * qi3)>>(DSCRT-1));
	att.R[1][2]=(qi2 * qi3 - qi0 * qi1)>>(DSCRT-1);
	att.R[2][0]=(qi1 * qi3 - qi0 * qi2)>>(DSCRT-1);
	att.R[2][1]=(qi2 * qi3 + qi0 * qi1)>>(DSCRT-1);
	att.R[2][2]=(1<<DSCRT) - ((qi1 * qi1 + qi2 * qi2)>>(DSCRT-1));
	att.q[0] = qi0;
	att.q[1] = qi1;
	att.q[2] = qi2;
	att.q[3] = qi3;
	if(sens.mag_updated){
		sens.mag_updated = 0;
		body2glob(m_bdy, m_glb, 3);
		m_ENU[0]=0;
		m_ENU[1]=sqrt(m_glb[0]*m_glb[0] + m_glb[1]*m_glb[1]);
	    m_ENU[2]=m_glb[2];
	}
}

void attitude_compute(void)
{
	static short turn=0;
	static int l_yaw=0;
	int pitch,roll,yaw;  //in (1<<14)rad
	marg_update();
	pitch = -asin(att.R[2][0]/DSCRT_F)*DSCRT_F;
	roll  = atan2(att.R[2][1], att.R[2][2])*DSCRT_F;
	yaw = -atan2(att.R[0][1], att.R[1][1])*DSCRT_F;
	//yaw changes continuously without jumping from -180 to 180
	if(yaw < (-(DSCRT_I*19/20)*PI) && l_yaw>((DSCRT_I*19/20)*PI))turn++;//0.95*16384=15565
	if(yaw > ((DSCRT_I*19/20)*PI) && l_yaw<(-(DSCRT_I*19/20)*PI))turn--;	
	l_yaw = yaw;
	yaw += turn*((DSCRT_I*2)*PI);	
	att.pitch=pitch;
	att.roll=roll;
	att.yaw=yaw;
}

#endif


float data_2_angle(float x, float y, float z)	 //in rad
{
	float res;
	res = atan2(x,sqrt(y*y+z*z));
	return res;
}
void quarternion_init(void)
{
	int i;
	long sum_ax=0,sum_ay=0,sum_az=0;
	long sum_mx=0,sum_my=0,sum_mz=0;
	short accx,accy,accz,magx,magy,magz; 
	float xh,yh;
	float pitch,roll,yaw; 
	int qi0,qi1,qi2,qi3;
	for(i=0;i<50;i++){
		continue_cps_read();
		delay_ms(8);
		sum_ax+=sens.ax;
		sum_ay+=sens.ay;
		sum_az+=sens.az;
		sum_mx+=sens.mx;
		sum_my+=sens.my;
		sum_mz+=sens.mz;
		
	}
	accx = sum_ax/50;
	accy = sum_ay/50;
	accz = sum_az/50;
	magx = sum_mx/50;
	magy = sum_my/50;
	magz = sum_mz/50;
	pitch = -data_2_angle(accx,accy,accz);//rad
	roll = data_2_angle(accy,accx,accz); //rad
	xh = magy*cos(roll)+magx*sin(roll)*sin(pitch)-magz*cos(pitch)*sin(roll);		 
	yh = magx*cos(pitch)+magz*sin(pitch);
	yaw = -atan2(xh,yh)+1.57f;

	if(yaw < -PI )
		yaw = yaw + 2*PI;
	if(yaw > PI )
		yaw = yaw - 2*PI;
	att.pitch = pitch*DSCRT_F;
	att.roll = roll*DSCRT_F;
	att.yaw = yaw*DSCRT_F;
	q0 = (cos(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) + sin(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw))*DSCRT_F;  
	q1 = (sin(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) - cos(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw))*DSCRT_F;    
	q2 = (cos(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw) + sin(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw))*DSCRT_F; 
	q3 = (cos(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw) - sin(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw))*DSCRT_F;  
	qi0=q0;
	qi1=q1;
	qi2=q2;
	qi3=q3;
	att.R[0][0]=(1<<DSCRT) - ((qi2 * qi2 + qi3 * qi3)>>(DSCRT-1));
	att.R[0][1]=(qi1 * qi2 - qi0 * qi3)>>(DSCRT-1);
	att.R[0][2]=(qi1 * qi3 + qi0 * qi2)>>(DSCRT-1);
	att.R[1][0]=(qi1 * qi2 + qi0 * qi3)>>(DSCRT-1);
	att.R[1][1]=(1<<DSCRT) - ((qi1 * qi1 + qi3 * qi3)>>(DSCRT-1));
	att.R[1][2]=(qi2 * qi3 - qi0 * qi1)>>(DSCRT-1);
	att.R[2][0]=(qi1 * qi3 - qi0 * qi2)>>(DSCRT-1);
	att.R[2][1]=(qi2 * qi3 + qi0 * qi1)>>(DSCRT-1);
	att.R[2][2]=(1<<DSCRT) - ((qi1 * qi1 + qi2 * qi2)>>(DSCRT-1));
	att.q[0] = q0;
	att.q[1] = q1;
	att.q[2] = q2;
	att.q[3] = q3;
}
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void body2glob(int body[], int glob[], short dimension)
{	
	if(dimension == 2){
		glob[0] = body[0]*cos(att.yaw/DSCRT_F) + body[1]*sin(-att.yaw/DSCRT_F);
		glob[1] = body[0]*sin(att.yaw/DSCRT_F) + body[1]*cos(att.yaw/DSCRT_F); 
	}
	else if(dimension == 3){
		glob[0] = (body[0]*att.R[0][0] + body[1]*att.R[0][1] + body[2]*att.R[0][2])>>DSCRT;
		glob[1] = (body[0]*att.R[1][0] + body[1]*att.R[1][1] + body[2]*att.R[1][2])>>DSCRT;
		glob[2] = (body[0]*att.R[2][0] + body[1]*att.R[2][1] + body[2]*att.R[2][2])>>DSCRT;
	}
}
void glob2body(int body[], int glob[], short dimension)//body=inv(R)*glob
{	
	if(dimension == 2){
		body[0] = glob[0]*cos(att.yaw/DSCRT_F) + glob[1]*sin(att.yaw/DSCRT_F);
		body[1] = glob[0]*sin(-att.yaw/DSCRT_F) + glob[1]*cos(att.yaw/DSCRT_F); 
	}
	else if(dimension == 3){
		body[0] = (glob[0]*att.R[0][0] + glob[1]*att.R[1][0] + glob[2]*att.R[2][0])>>DSCRT;
		body[1] = (glob[0]*att.R[0][1] + glob[1]*att.R[1][1] + glob[2]*att.R[2][1])>>DSCRT;
		body[2] = (glob[0]*att.R[0][2] + glob[1]*att.R[1][2] + glob[2]*att.R[2][2])>>DSCRT;
	}
}

