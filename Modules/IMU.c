#include "../Main/global.h"
#include "IMU.h"
#include "../Main/timer.h"
#include "../Drivers/TWI.h"
#include "Filter.h"
IIRFilter iir_ax={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_ay={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_az={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
/*IIRFilter iir_gx={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_gy={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_gz={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
*/
char compass_received[6];

short gyr_x_bias=0;
short gyr_y_bias=0;
short gyr_z_bias=0;
short ax,ay,az;
#if BOARD_V5
	#if F330
	short acc_x_bias=0;
	short acc_y_bias=0;
	short acc_z_bias=-500;
	short mag_x_bias=-60;
	short mag_y_bias=-30;
	short mag_z_bias=-35;

	#elif F450
	short acc_x_bias=90;
	short acc_y_bias=56;
	short acc_z_bias=145;
	short mag_x_bias=-50;
	short mag_y_bias=30;
	short mag_z_bias=-30;

	#elif XINSONG
	short acc_x_bias=-50;
	short acc_y_bias=140;
	short acc_z_bias=-366;
	short mag_x_bias=-20;
	short mag_y_bias=30;
	short mag_z_bias=-160;

	#endif
#endif

void gyro_calibration(void)
{
	long temp1=0,temp2=0,temp3=0;
	int i;
	gyr_x_bias=0;gyr_y_bias=0;gyr_z_bias=0;
	for(i=0;i<200;i++)
	{
		temp1+=sens.gx;
		temp2+=sens.gy;
		temp3+=sens.gz;
		delay_ms(5);
	}
	gyr_x_bias=-temp1/200;
	gyr_y_bias=-temp2/200;
	gyr_z_bias=-temp3/200;
}

void continue_cps_read(void)
{
	twi_cps_read_start(compass_received);
}

void data_conclude(char switcher)
{
	typedef union myun{
		char b[2];
		short c;
	}un;
	un x_get,y_get,z_get;
	short temp;
	switch (switcher){
	case ACC_SWITCH:
		break;
	case GYRO_SWITCH:	
		break;
	case CPS_SWITCH:
		x_get.b[1] = compass_received[0];
		x_get.b[0] = compass_received[1];
		y_get.b[1] = compass_received[2];
		y_get.b[0] = compass_received[3];
		z_get.b[1] = compass_received[4];
		z_get.b[0] = compass_received[5];
		temp = z_get.c;
		z_get.c = -y_get.c;	
		y_get.c = -x_get.c;
		x_get.c = -temp;
		if(x_get.c>-800&&x_get.c<800)
			sens.mx = x_get.c + mag_x_bias;
		if(y_get.c>-800&&y_get.c<800)
			sens.my = y_get.c + mag_y_bias;
		if(z_get.c>-800&&z_get.c<800)
			sens.mz = z_get.c + mag_z_bias;
		break;
	default:
		break;
	}
}
void acc_lowpass_biascorr(short ax, short ay, short az)
{
	sens.ax = IIR_apply(&iir_ax, ax)+acc_x_bias;
	sens.ay = IIR_apply(&iir_ay, ay)+acc_y_bias;
	sens.az = IIR_apply(&iir_az, az)+acc_z_bias;
}
void gyro_lowpass_biascorr(short gx, short gy, short gz, unsigned char store)
{
	static short sx=0,sy=0,sz=0;
	if(store){
		sx = gx;
		sy = gy;
		sz = gz;
	}
	else{
	//	sens.gx = IIR_apply(&iir_gx, (gx+sx)/2)+gyr_x_bias;
	//	sens.gy = IIR_apply(&iir_gy, (gy+sy)/2)+gyr_y_bias;//
	//	sens.gz = IIR_apply(&iir_gz, (gz+sz)/2)+gyr_z_bias;//(gz+sz)/2+gyr_z_bias;
		sens.gx = (gx+sx)/2+gyr_x_bias;
		sens.gy = (gy+sy)/2+gyr_y_bias;
		sens.gz = (gz+sz)/2+gyr_z_bias;
	}
}
void imu_IIR_init(void)
{	
	float cutoff_freq = 15.0;//, cutoff_freq2 = 40.0;
	float smpl_freq = 500.0;
	IIR_set_cutoff_freq(&iir_ax, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_ay, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_az, 8.0, smpl_freq);
	sens.ax = IIR_reset(&iir_ax, 0);
	sens.ay = IIR_reset(&iir_ay, 0);
	sens.az = IIR_reset(&iir_az, 8192);
/*	IIR_set_cutoff_freq(&iir_gx, cutoff_freq2, smpl_freq);
	IIR_set_cutoff_freq(&iir_gy, cutoff_freq2, smpl_freq);
	IIR_set_cutoff_freq(&iir_gz, cutoff_freq2, smpl_freq);
	sens.gx = IIR_reset(&iir_gx, 0);
	sens.gy = IIR_reset(&iir_gy, 0);
	sens.gz = IIR_reset(&iir_gz, 0);
	*/
}
