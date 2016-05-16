#include "../Main/global.h"
#include "../Main/commons.h"
#include "AttitudeEstimator.h"
#include "math.h"
#include "Controls.h"
#define VEL_FF_XY_P 0.6
#define VEL_FF_Z_P 0.5

#if F330
	#if INDOOR
	PID pitchPID={0,0,0,0,
			2.1,
			14.0,1.8,0.005};
	PID rollPID={0,0,0,0,
			2.1,
			14.0,1.8,0.005};
	PID yawPID={0,0,0,0,
			1.7,
			13.0,0,0};
	PID altPID={0,0,0,0,
			1.42,
			1.48,0.07,0.0};
	PID pos_xPID={0,0,0,0,
			0.22,
			2.05,0.4,0};
	PID pos_yPID={0,0,0,0,
			0.22,
			2.05,0.4,0};
	#elif OUTDOOR
	PID pitchPID={0,0,0,0,
			1.5,
			12.5,1.8,0.0};
	PID rollPID={0,0,0,0,
			1.5,
			12.5,1.8,0.0};
	PID yawPID={0,0,0,0,
			1.3,
			10.0,0.5,0};
	PID altPID={0,0,0,0,
			2.2,
			2.38,0.07,0.0};
	PID pos_xPID={0,0,0,0,
			0.42,
			3.3,0.2,0.1};
	PID pos_yPID={0,0,0,0,
			0.42,
			3.3,0.2,0.1};
	#endif		
			
#elif F450
	#if INDOOR
	PID pitchPID={0,0,0,0,
			2.33,
			23.8,3.2,0.05};
	PID rollPID={0,0,0,0,
			2.33,
			23.8,3.2,0.05};
	PID yawPID={0,0,0,0,
			1.2,
			11.0,1.0,0};
	PID altPID={0,0,0,0,
			1.93,
			1.81,0.07,0.0};
	PID pos_xPID={0,0,0,0,
			0.22,
			2.05,0.0,0};
	PID pos_yPID={0,0,0,0,
			0.22,
			2.05,0.0,0};
	#elif OUTDOOR
	PID pitchPID={0,0,0,0,
			1.6,
			16.0,0.8,0.0};
	PID rollPID={0,0,0,0,
			1.6,
			16.0,0.8,0.0};
	PID yawPID={0,0,0,0,
			0.75,
			10.0,0.0,0.0};
	PID altPID={0,0,0,0,
			2.0,
			1.9,0.02,0.0};
	PID pos_xPID={0,0,0,0,
			0.37,
			2.5,0.05,0.0};
	PID pos_yPID={0,0,0,0,
			0.37,
			2.5,0.05,0.0};
	#endif
#elif XINSONG
	#if INDOOR
	PID pitchPID={0,0,0,0,
			2.5,
			21.0,2.0,0.005};
	PID rollPID={0,0,0,0,
			2.5,
			21.0,2.0,0.005};
	PID yawPID={0,0,0,0,
			1.2,
			11.0,0,0};
	PID altPID={0,0,0,0,
			2.26,
			1.79,0.07,0.0};
	PID pos_xPID={0,0,0,0,
			0.22,
			2.05,0.06,0.06};
	PID pos_yPID={0,0,0,0,
			0.22,
			2.05,0.06,0.06};
	#elif OUTDOOR
	PID pitchPID={0,0,0,0,
			2.5,
			21.0,2.0,0.005};
	PID rollPID={0,0,0,0,
			2.5,
			21.0,2.0,0.005};
	PID yawPID={0,0,0,0,
			1.2,
			11.0,0,0};
	PID altPID={0,0,0,0,
			2.26,
			1.99,0.07,0.0};
	PID pos_xPID={0,0,0,0,
			0.191,
			1.34,0.0,0.12};
	PID pos_yPID={0,0,0,0,
			0.22,
			1.34,0.0,0.12};
	#endif						
#elif F240
PID pitchPID={0,0,0,0,
			1.1,
			11.4,1.0,0.0};
PID rollPID={0,0,0,0,
			1.40,
			11.8,1.0,0.0};
PID yawPID={0,0,0,0,
			1.264,
			10.0,0.4,0};
PID altPID={0,0,0,0,
			0.84,
			1.69,0.0,0.0};
PID pos_xPID={0,0,0,0,
			0.30,
			2.65,0.0,0.006};
PID pos_yPID={0,0,0,0,
			0.30,
			2.65,0.0,0.006};
#endif
int external_err_pid(PID *pid, int err, short dt);
int internal_err_pid(PID *pid, int rate_err, short dt);
int R_sp[3][3];
void position_control(short dt)
{
	int globAcc_sp[3], Z_b[3], y_C[3];
	int body_x_sp[3], body_y_sp[3], body_z_sp[3];
	float norm;
	short i;	
	//get acc sp from position setpoint
	cmd.vel_x_sp = constrain(external_err_pid(&pos_xPID, cmd.pos_x_sp - pos.x_est[0], dt), -MAX_XY_VEL, MAX_XY_VEL);
	cmd.vel_y_sp = constrain(external_err_pid(&pos_yPID, cmd.pos_y_sp - pos.y_est[0], dt), -MAX_XY_VEL, MAX_XY_VEL);
	cmd.vel_z_sp = constrain(external_err_pid(&altPID, cmd.pos_z_sp - pos.z_est[0], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
	cmd.vel_x_sp +=  VEL_FF_XY_P * cmd.vel_x_ff;
	cmd.vel_y_sp +=  VEL_FF_XY_P * cmd.vel_y_ff;
	cmd.vel_z_sp +=  VEL_FF_Z_P * cmd.vel_z_ff;
//	data2[4] = globVel_sp[0];
//	data2[6] = globVel_sp[1];
	globAcc_sp[0] = internal_err_pid(&pos_xPID, cmd.vel_x_sp - pos.x_est[1], dt);
	globAcc_sp[1] = internal_err_pid(&pos_yPID, cmd.vel_y_sp - pos.y_est[1], dt);
	globAcc_sp[2] = GRAVITY + internal_err_pid(&altPID, cmd.vel_z_sp - pos.z_est[1], dt);
	//dot product of setpoint and current z axis of body
	for(i=0;i<3;i++)
		Z_b[i]=att.R[i][2];
//in c++,	output.thrustForce = (globAcc_sp * Z_b) * VEHICLE_MASS;
	output.thrustForce = ((Z_b[0] * globAcc_sp[0] + Z_b[1] * globAcc_sp[1] + Z_b[2] * globAcc_sp[2])>>DSCRT) * VEHICLE_MASS/1000 ;
	//get the last col of R_sp from acc setpoint
	for(i=0;i<3;i++)
		body_z_sp[i] = globAcc_sp[i];
//in c++,	body_z_sp.normalize();
	norm = sqrt(body_z_sp[0] * body_z_sp[0] + body_z_sp[1] * body_z_sp[1] + body_z_sp[2] * body_z_sp[2]);
	for(i=0;i<3;i++)
		body_z_sp[i] = (body_z_sp[i]<<DSCRT) / norm;
	//according to yaw_sp get the rest of R_sp
	y_C[0] = -sin(cmd.yaw_sp/DSCRT_F) * DSCRT_F;
	y_C[1] = cos(cmd.yaw_sp/DSCRT_F) * DSCRT_F;
	y_C[2] = 0;
//c = a % c
//rc = (ra % rb)/r
//in c++	body_x_sp = y_C % body_z_sp;
	body_x_sp[0] = (y_C[1] * body_z_sp[2] - y_C[2] * body_z_sp[1])>>DSCRT;
	body_x_sp[1] = (y_C[2] * body_z_sp[0] - y_C[0] * body_z_sp[2])>>DSCRT;
	body_x_sp[2] = (y_C[0] * body_z_sp[1] - y_C[1] * body_z_sp[0])>>DSCRT;
//in c++	body_x_sp.normalize();
	norm = sqrt(body_x_sp[0] * body_x_sp[0] + body_x_sp[1] * body_x_sp[1] + body_x_sp[2] * body_x_sp[2]);
	for(i=0;i<3;i++)
		body_x_sp[i] = (body_x_sp[i]<<DSCRT) / norm;	
//in c++	body_y_sp = body_z_sp % body_x_sp;
	body_y_sp[0] = (body_z_sp[1] * body_x_sp[2] - body_z_sp[2] * body_x_sp[1])>>DSCRT;
	body_y_sp[1] = (body_z_sp[2] * body_x_sp[0] - body_z_sp[0] * body_x_sp[2])>>DSCRT;
	body_y_sp[2] = (body_z_sp[0] * body_x_sp[1] - body_z_sp[1] * body_x_sp[0])>>DSCRT;
	for (i = 0; i < 3; i++) {
		R_sp[i][0] = body_x_sp[i];
		R_sp[i][1] = body_y_sp[i];
		R_sp[i][2] = body_z_sp[i];
	}
	cmd.pitch_sp = -asin(R_sp[2][0]/DSCRT_F)*DSCRT_F;
	cmd.roll_sp = atan2(R_sp[2][1], R_sp[2][2])*DSCRT_F;
}
void altitude_control(short dt)
{
	int altAcc_sp;
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC){
	//pass command directly into output				
		output.thrustForce = cmd.throttle * GRAVITY/1000 * VEHICLE_MASS>>9;//0~1024 -> 0~20,000 mN, put 1000 forward or overflows	
	}
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL){	
		cmd.vel_z_sp = constrain(external_err_pid(&altPID, cmd.pos_z_sp - pos.z_est[0], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
		cmd.vel_z_sp +=  VEL_FF_Z_P * cmd.vel_z_ff;
		altAcc_sp = GRAVITY + internal_err_pid(&altPID, cmd.vel_z_sp - pos.z_est[1], dt);
		output.thrustForce = ((altAcc_sp<<DSCRT) / att.R[2][2]) * VEHICLE_MASS/1000;
	}
}
void manual_R_sp_generate(void)
{
	float cp = cos(cmd.pitch_sp/DSCRT_F);
	float sp = sin(cmd.pitch_sp/DSCRT_F);
	float sr = sin(cmd.roll_sp/DSCRT_F);
	float cr = cos(cmd.roll_sp/DSCRT_F);
	float sy = sin(cmd.yaw_sp/DSCRT_F);
	float cy = cos(cmd.yaw_sp/DSCRT_F);
	R_sp[0][0] = cp * cy * DSCRT_F;
	R_sp[0][1] = ((sr * sp * cy) - (cr * sy)) * DSCRT_F;
	R_sp[0][2] = ((cr * sp * cy) + (sr * sy)) * DSCRT_F;
	R_sp[1][0] = cp * sy * DSCRT_F;
	R_sp[1][1] = ((sr * sp * sy) + (cr * cy)) * DSCRT_F;
	R_sp[1][2] = ((cr * sp * sy) - (sr * cy)) * DSCRT_F;
	R_sp[2][0] = -sp * DSCRT_F;
	R_sp[2][1] = sr * cp * DSCRT_F;
	R_sp[2][2] = cr * cp * DSCRT_F;
}
void Qsp2Rsp(void)
{
	R_sp[0][0]=(1<<DSCRT) - ((ctrl.rasp_q_sp[2] * ctrl.rasp_q_sp[2] + ctrl.rasp_q_sp[3] * ctrl.rasp_q_sp[3])>>(DSCRT-1));
	R_sp[0][1]=(ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[2] - ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[3])>>(DSCRT-1);
	R_sp[0][2]=(ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[3] + ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[2])>>(DSCRT-1);
	R_sp[1][0]=(ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[2] + ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[3])>>(DSCRT-1);
	R_sp[1][1]=(1<<DSCRT) - ((ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[1] + ctrl.rasp_q_sp[3] * ctrl.rasp_q_sp[3])>>(DSCRT-1));
	R_sp[1][2]=(ctrl.rasp_q_sp[2] * ctrl.rasp_q_sp[3] - ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[1])>>(DSCRT-1);
	R_sp[2][0]=(ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[3] - ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[2])>>(DSCRT-1);
	R_sp[2][1]=(ctrl.rasp_q_sp[2] * ctrl.rasp_q_sp[3] + ctrl.rasp_q_sp[0] * ctrl.rasp_q_sp[1])>>(DSCRT-1);
	R_sp[2][2]=(1<<DSCRT) - ((ctrl.rasp_q_sp[1] * ctrl.rasp_q_sp[1] + ctrl.rasp_q_sp[2] * ctrl.rasp_q_sp[2])>>(DSCRT-1));
	cmd.pitch_sp = -asin(R_sp[2][0]/DSCRT_F)*DSCRT_F;
	cmd.roll_sp  = atan2(R_sp[2][1], R_sp[2][2])*DSCRT_F;
	cmd.yaw_sp = -atan2(R_sp[0][1], R_sp[1][1])*DSCRT_F;
}
void set_thrust_force(void)
{
	output.thrustForce = ctrl.rasp_thrust;
}
void reset_variables(void)
{
	pitchPID.int_RateErr = 0;
	rollPID.int_RateErr = 0;
	yawPID.int_RateErr = 0;
	altPID.int_RateErr = 0;
	pos_xPID.int_RateErr = 0;
	pos_yPID.int_RateErr = 0;
	cmd.yaw_sp = att.yaw;
	cmd.pos_z_sp = pos.z_est[0] / 1000;
}
void attitude_control(short dt)
{
	//R_sp 	world to desired body
	//att.R 		world to measured body
	int e_R_hat[3][3];//measured body to desired body
	int e_R[3], e_w[3];
	int w_sp[3];
//in c++	e_R_hat = (R_sp.transposed() * R - R.transposed() * R_sp) * (0.5f);
//	e_R_hat[0][0] = R_sp[0][0] * att.R[0][0] + R_sp[1][0] * att.R[1][0] + R_sp[2][0] * att.R[2][0];
	e_R_hat[0][1] = R_sp[0][0] * att.R[0][1] + R_sp[1][0] * att.R[1][1] + R_sp[2][0] * att.R[2][1];
	e_R_hat[0][2] = R_sp[0][0] * att.R[0][2] + R_sp[1][0] * att.R[1][2] + R_sp[2][0] * att.R[2][2];
	e_R_hat[1][0] = R_sp[0][1] * att.R[0][0] + R_sp[1][1] * att.R[1][0] + R_sp[2][1] * att.R[2][0];
//	e_R_hat[1][1] = R_sp[0][1] * att.R[0][1] + R_sp[1][1] * att.R[1][1] + R_sp[2][1] * att.R[2][1];
	e_R_hat[1][2] = R_sp[0][1] * att.R[0][2] + R_sp[1][1] * att.R[1][2] + R_sp[2][1] * att.R[2][2];	
	e_R_hat[2][0] = R_sp[0][2] * att.R[0][0] + R_sp[1][2] * att.R[1][0] + R_sp[2][2] * att.R[2][0];
	e_R_hat[2][1] = R_sp[0][2] * att.R[0][1] + R_sp[1][2] * att.R[1][1] + R_sp[2][2] * att.R[2][1];
//	e_R_hat[2][2] = R_sp[0][2] * att.R[0][2] + R_sp[1][2] * att.R[1][2] + R_sp[2][2] * att.R[2][2];
	
//	e_R_hat[0][0] -= att.R[0][0] * R_sp[0][0] + att.R[1][0] * R_sp[1][0] + att.R[2][0] * R_sp[2][0];
	e_R_hat[0][1] -= att.R[0][0] * R_sp[0][1] + att.R[1][0] * R_sp[1][1] + att.R[2][0] * R_sp[2][1];
	e_R_hat[0][2] -= att.R[0][0] * R_sp[0][2] + att.R[1][0] * R_sp[1][2] + att.R[2][0] * R_sp[2][2];
	e_R_hat[1][0] -= att.R[0][1] * R_sp[0][0] + att.R[1][1] * R_sp[1][0] + att.R[2][1] * R_sp[2][0];
//	e_R_hat[1][1] -= att.R[0][1] * R_sp[0][1] + att.R[1][1] * R_sp[1][1] + att.R[2][1] * R_sp[2][1];
	e_R_hat[1][2] -= att.R[0][1] * R_sp[0][2] + att.R[1][1] * R_sp[1][2] + att.R[2][1] * R_sp[2][2];
	e_R_hat[2][0] -= att.R[0][2] * R_sp[0][0] + att.R[1][2] * R_sp[1][0] + att.R[2][2] * R_sp[2][0];
	e_R_hat[2][1] -= att.R[0][2] * R_sp[0][1] + att.R[1][2] * R_sp[1][1] + att.R[2][2] * R_sp[2][1];
//	e_R_hat[2][2] -= att.R[0][2] * R_sp[0][2] + att.R[1][2] * R_sp[1][2] + att.R[2][2] * R_sp[2][2];
		
//	e_R_hat[0][0] = e_R_hat[0][0]>>(DSCRT+1);
	e_R_hat[0][1] = e_R_hat[0][1]>>(DSCRT+1);
	e_R_hat[0][2] = e_R_hat[0][2]>>(DSCRT+1);
	e_R_hat[1][0] = e_R_hat[1][0]>>(DSCRT+1);
//	e_R_hat[1][1] = e_R_hat[1][1]>>(DSCRT+1);
	e_R_hat[1][2] = e_R_hat[1][2]>>(DSCRT+1);
	e_R_hat[2][0] = e_R_hat[2][0]>>(DSCRT+1);
	e_R_hat[2][1] = e_R_hat[2][1]>>(DSCRT+1);
//	e_R_hat[2][2] = e_R_hat[2][2]>>(DSCRT+1);
	
//in c++	math::Vector<3> e_R
	e_R[0] = (e_R_hat[1][2] - e_R_hat[2][1])>>1;//roll err in rad<<14
	e_R[1] = (e_R_hat[2][0] - e_R_hat[0][2])>>1;//pitch err in rad<<14
	e_R[2] = (e_R_hat[0][1] - e_R_hat[1][0])>>1;//yaw err in rad<<14

	w_sp[0] = external_err_pid(&rollPID, e_R[0], dt);//RollRate_sp
	w_sp[1] = external_err_pid(&pitchPID, e_R[1], dt);//PitchRate_sp
	w_sp[2] = external_err_pid(&yawPID, e_R[2], dt);//YawRate_sp
	

	e_w[0] = w_sp[0] - att.rollspeed;
	e_w[1] = w_sp[1] - att.pitchspeed;
	e_w[2] = w_sp[2] - att.yawspeed;
	e_w[0] = e_w[0] >> (DSCRT-14);//the params are tuned when DSCRT = 14
	e_w[1] = e_w[1] >> (DSCRT-14);
	e_w[2] = e_w[2] >> (DSCRT-14);	
	output.rollMmt = internal_err_pid(&rollPID, e_w[0], dt);
	output.pitchMmt = internal_err_pid(&pitchPID, e_w[1], dt);
	output.yawMmt = internal_err_pid(&yawPID, e_w[2], dt);
}

int external_err_pid(PID *pid, int err, short dt)
{
 	 int retValue;								
 	 pid->Err = err;
 	 retValue = pid->P * err;
	 return retValue;
}
int internal_err_pid(PID *pid, int rate_err, short dt)
{
   	int rateOuput;								
  	pid->RateErr = rate_err;
  	rateOuput = pid->Prate * rate_err + pid->Drate * (rate_err - pid->l_RateErr) *1000/ dt + pid->Irate * pid->int_RateErr / 1000;
  	pid->l_RateErr = rate_err;
	pid->int_RateErr += rate_err * dt;
  	return rateOuput;
}

