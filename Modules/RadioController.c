#include "RadioController.h"
#include "../Main/global.h"
#include "AttitudeEstimator.h"
#include "../Main/commons.h"
void command_init(void)
{
	cmd.pos_z_sp = pos.z_est[0] / 1000;	
	cmd.pos_x_sp = pos.x_est[0] / 1000;
	cmd.pos_y_sp = pos.y_est[0] / 1000;	
}
void get_rc(short dt)
{
	int expctBodyVel[2];
	int expctGlobVel[2];
	int expctAltRate,expctYawRate;
	//mode change
	mode.l_FlightMode = mode.FlightMode;
	
	#if OFFBOARD_AVAIL
	if (cmd.rc[5] > -307 || myusb.rcv_timeout == 0){//on board
		mode.offboard = 0;
		if (cmd.rc[4] > 307){//switch upward	
			mode.FlightMode = ALT_CTRL;	
		}
		else if(cmd.rc[4] > -307){		//in the middle
			mode.FlightMode = ALT_CTRL;
		}
		else{		//downward
			mode.FlightMode = MANUEL;
		}
	}
	else{//rasp
		mode.offboard = 1;
		if (cmd.rc[4] > 307){//switch upward	
			mode.FlightMode = RASP_POS;	
		}
		else if(cmd.rc[4] > -307){		//in the middle
			mode.FlightMode = RASP_ALT;
		}
		else{		//downward
			mode.FlightMode = RASP_MANUEL;
		}
	}
	#else
	mode.offboard = 0;
	if (cmd.rc[4] > 307){//switch upward	
		mode.FlightMode = POS_CTRL;	
	}
	else if(cmd.rc[4] > -307){		//in the middle
		mode.FlightMode = ALT_CTRL;
	}
	else{		//downward
		mode.FlightMode = MANUEL;
	}

	#endif
	mode.l_CalMode = mode.CalibrationMode;
	mode.CalibrationMode = 0;
	//pitch roll att command for acrob, manuel, altctrl
	if(mode.FlightMode == MANUEL || mode.FlightMode==ACROBATIC || mode.FlightMode == ALT_CTRL){
		cmd.pitch_sp = dead_zone(cmd.rc[1] * MAX_ATT_MANUEL>>10, 0);
		cmd.roll_sp = dead_zone(-cmd.rc[0] * MAX_ATT_MANUEL>>10, 0);
	}
	//xy pos command for posctrl
	else if(mode.FlightMode == POS_CTRL){
		expctBodyVel[0] = dead_zone(cmd.rc[1] * MAX_XY_VEL_MANUEL>>10, 400);
		expctBodyVel[1] = dead_zone(cmd.rc[0] * MAX_XY_VEL_MANUEL>>10, 400);
		body2glob(expctBodyVel, expctGlobVel, 2);
		cmd.pos_x_sp += expctGlobVel[0] * dt / 1000;	
		cmd.pos_y_sp += expctGlobVel[1] * dt / 1000;
		cmd.vel_x_ff = expctGlobVel[0];
		cmd.vel_y_ff = expctGlobVel[1];
	}												
	//yaw command for all
	expctYawRate = dead_zone(cmd.rc[3] * MAX_YAW_RATE_MANEUL>>10, 4260);//deadzone 15 deg
	cmd.yaw_sp += expctYawRate * dt>>10;		
	//throtle command for manuel, acrob
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC){
		cmd.throttle = dead_zone(constrain((cmd.rc[2] + 1024)/2 * thrCmndRatio, 0, 1400), 50);
	}
	//alt command for altctrl, posctrl
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL){
		expctAltRate = dead_zone(cmd.rc[2] * MAX_ALT_VEL_MANUEL>>10, 200);
		cmd.pos_z_sp += expctAltRate * dt / 1000;
		cmd.vel_z_ff = expctAltRate;
	}
	//tuning
	
/*
*
*channel 5 no more usable for tuning in usb mode!
*
*/	
	
//	pos_xPID.Prate = 2.5 + (cmd.rc[5]/1024.0)*1.0;
//	pos_xPID.P = 0.3 + (cmd.rc[6]/1024.0)*0.15;
//	pos_xPID.Drate = 0.1 + (cmd.rc[7]/1024.0)*0.1;
	
//	pos_yPID.P = pos_xPID.P;
//	pos_yPID.Prate = pos_xPID.Prate;	
//	pos_yPID.Irate = pos_xPID.Irate;
//	pos_yPID.Drate = pos_xPID.Drate;	
//	altPID.Prate = 1.8 + (cmd.rc[5]/1024.0)*0.7;
//	altPID.P = 1.9 + (cmd.rc[6]/1024.0)*0.7;
//	altPID.Irate = 0.05 + (cmd.rc[7]/1024.0)*0.05;
	
//	pitchPID.Prate = 22.5 + (cmd.rc[5]/1024.0)*8.0;
//	pitchPID.Drate = 0.03 + (cmd.rc[6]/1024.0)*0.03;
//	pitchPID.P = 2.1 + (cmd.rc[6]/1024.0)*1.0;
//	pitchPID.Irate = 1.8 + (cmd.rc[7]/1024.0)*1.8;
//	rollPID.Prate = 11.0 + (cmd.rc[5]/1024.0)*9.0;
//	rollPID.Drate = 0.03 + (cmd.rc[7]/1024.0)*0.03;
//	rollPID.Prate = pitchPID.Prate;
//	rollPID.P = pitchPID.P;
//	rollPID.Irate = pitchPID.Irate;
//	yawPID.Prate = 10.0 + (cmd.rc[5]/1024.0)*8.0;
//	yawPID.P = 1.2 + (cmd.rc[6]/1024.0)*0.9;
}
