/*
 *	Flight Control Program
 *	Processor: AT91SAM7S256
 *	Board Version: 3.3
 *	June, 2014 - June, 2016
 *	Author: Siyao HU, Tong WU, Jiahao FANG, Xin YE
 */
#include "timer.h"
#include "global.h"

#include "../Drivers/PWM.h"
#include "../Drivers/USB.h"
#include "../Drivers/SPI.h"
#include "../Drivers/TWI.h"
#include "../Drivers/ADC.h"
#include "../Drivers/PIO.h"
#include "../Drivers/Pins.h"
#include "../Drivers/PIT.h"
#include "../Drivers/GPS.h"
#include "../Drivers/XBee.h"

#include "../Devices/MS5611_SPI.h"
#include "../Devices/HMC5883.h"
#include "../Devices/MPU6000_SPI.h"
#include "../Devices/EEPROM.h"
#include "../Devices/PCA9685.h"

#include "../Modules/IMU.h"
#include "../Modules/AttitudeEstimator.h"
#include "../Modules/PositionEstimator.h"
#include "../Modules/Controls.h"
#include "../Modules/RadioController.h"
#include "../Modules/MotorMixer.h"



void init_loop(void);
int self_check(void);
void data_select(void);
void ground_work_once(void);
void Process500Hz(void);
void Process250Hz_A(void);
void Process250Hz_B(void);
void Process50Hz(void);
#if OUTDOOR
struct _gps gps = {0,0,
				0,0,0,
				0,0,0,
				0,0,0,
				0,0,0};
#elif INDOOR
struct _vicon vicon = {0,0,0,0,0,0,0};
#endif
struct _smpl smpl = {0,0,1,0,0};
struct _att att = {0,0,0,
				0,0,0,
				{{DSCRT_I,0,0},
				 {0,DSCRT_I,0},
				 {0,0,DSCRT_I}},
				 {0,0,0,0}};
struct _sens sens = {0,0,0,
					0,0,0,
					0,0,0,0,0};
struct _mode mode = {MANUEL,0,MANUEL,0,0,1};
struct _baro baro = {0,0,0,0,0};
struct _pos pos = {{0,0},{0,0},{0,0},
					0,0,0,
					0};
struct _cmd cmd = {{0,0,-1024,0,0,0,0,0,0},
					0,0,0,
					0,0,0,
					0,0,0,
					0,0,0,
					0,SonarOFF,sendPOS};
struct _ctrl ctrl = {{DSCRT_I,0,0,0},0};
struct _output output = {0,0,0,0};
struct _adc adc = {0};
struct _AT91S_CDC 	pCDC;
struct _myusb myusb = {0,0,{0,0,0},0,0,0};
short data2[9]={0,0,0,0,0,0,0,0,0};
unsigned int process_count=0;
unsigned int l_uart_time=0;
unsigned int uart_time=0;
short uart_dt;
short idle_time=0;

void data_select(void)
{
	switch(cmd.data2send){
	case sendNON:
		break;
	case sendSENS://1
		data2[0]=sens.ax;
		data2[1]=sens.ay;		
		data2[2]=sens.az;		
		data2[3]=sens.gx;
		data2[4]=sens.gy;
		data2[5]=sens.gz;
		data2[6]=sens.mx;
		data2[7]=sens.my;
		data2[8]=sens.mz;
		break;
	case sendGPS://2
	#if OUTDOOR
		data2[0] = gps.x;
		data2[1] = gps.y;		
		data2[2] = gps.z;				
		data2[3] = gps.vx;		//sends in unit 0.1 deg
		data2[4] = gps.vy;	
		data2[5] = 0;		
		data2[6] = gps.azm*57.3;
		data2[7] = gps.sat;
		data2[8] = gps.vel;
	#endif	
		break;
	case sendATT://3
		data2[0] = pos.Acc_x;
		data2[1] = pos.Acc_y;//att.pitch*573>>DSCRT;		
		data2[2] = pos.Acc_z;//att.yaw*573>>DSCRT;				
		data2[3] = att.roll*573>>DSCRT;
		data2[4] = att.pitch*573>>DSCRT;
		data2[5] = 0;
		data2[6] = cmd.yaw_sp*573>>DSCRT;//pos.Acc_x;//cmd.roll_sp*573>>DSCRT;att.rollspeed*573>>DSCRT;
		data2[7] = att.yaw*573>>DSCRT;	//pos.Acc_y;//cmd.pitch_sp*573>>DSCRT;	
		data2[8] = att.yawspeed*573>>DSCRT;//cmd.yaw_sp*573>>DSCRT;
		break;
	case sendPOS://4
		data2[0] = pos.x_est[0] / 1000;
		data2[1] = pos.y_est[0] / 1000;//pos.y_est[0] / 1000;
		data2[2] = pos.z_est[0] / 1000;//cmd.pitch_sp*573>>DSCRT;			
		data2[3] = pos.x_est[1] / 1000;//cmd.pos_y_sp
		data2[4] = pos.y_est[1] / 1000;
		data2[5] = pos.z_est[1] / 1000;
		data2[6] = pos.Acc_x;//cmd.pos_x_sp;
		data2[7] = pos.Acc_y;
		data2[8] = pos.Acc_z;

		break;
	case sendPID://5
		break;
		case sendCMD://6
		data2[0]=cmd.rc[0];
		data2[1]=cmd.rc[1];		
		data2[2]=cmd.rc[2];				
		data2[3]=cmd.rc[3];
		data2[4]=cmd.rc[4];
		data2[5]=cmd.rc[5];
		data2[6]=idle_time;
		data2[7]=l_uart_time;
		data2[8]=uart_dt;;
		

		break;
	case sendOUT://7
		data2[0]=att.pitchspeed*573>>DSCRT;
		data2[1]=output.pitchMmt/10;//cmd.pitch_sp*573>>DSCRT;		
		data2[2]=att.rollspeed*573>>DSCRT;				
		data2[3]=output.rollMmt/10;
		data2[4]=att.yawspeed*573>>DSCRT;
		data2[5]=output.yawMmt/10;
		data2[6]=output.thrustForce/10;;//cmd.Thrust;
		data2[7]=0;
		data2[8]=0;
		break;
	default:
		break;
	}
}
int main (void)
{		
	pio_init();
	led_ctrl(LED1,OFF);
	led_ctrl(LED2,ON);
	led_ctrl(LED3,OFF);
	pwm_init();
	motor_cut();
	pit_init();		
	ppm_clock_init();
	timer_init();
	
	USB_init();
	if(USB_armed()){
		myusb.connect_flag = PLUG_IN;
	}
	USB_check();
	
	xbee_init();
#if OUTDOOR
	gps_init();
#endif
//	delay_ms(100);
	twi_init();
	hmc5883_config();
	
	#if PWM16
	pca_init();
	#endif
	
	twi_fast_init();

	spi_init();
	MS5611_init();
	mpu6000_config();
	
	imu_IIR_init();
	rate_IIR_init(100);
	
	spi_fast_init();
	smpl.sens_rdy =1;
//	USB_init();
	adc_init();
	adc_start_conversion();
//	if(USB_armed()){
//		myusb.connect_flag = PLUG_IN;
//	}
//	USB_check();
	beep(ON);
	delay_ms(1000);
	beep(OFF);
	led_ctrl(LED2,OFF);
	led_ctrl(LED1,ON);
	/*wait until the vehicle is stablized for calibration*/		
	while(1){
	/*The following 3 initiations must be done 
	when the velcle is in static state, but not 
	required to be placed horizontally*/
		//wait for unlock	
		gyro_calibration();		
		quarternion_init();		
		init_loop();
#if ON_FLIGHT
#if USB_TEST
#else
		while(self_check());
#endif
#endif
		led_ctrl(LED1, OFF);
		beep(OFF);
//		if(USB_armed()){
//			myusb.connect_flag = 1;
//		}	  
		while(1){//flight loop		
		/*deal with attitude (and position prediction), xbee send, gps (and xy position correction),
		radio control (and motor output), baro (and altitude correction)*/												
			if(smpl.Flag500Hz){
				smpl.Flag500Hz = 0;
				Process500Hz();
				switch(process_count){
				case 0:
					Process50Hz();
				case 2: case 4: case 6: case 8:
					Process250Hz_B();
					break;
				case 1: case 3: case 5: case 7: case 9:
					Process250Hz_A();
					break;
				default:
					break;
				}
				process_count++;
				if(process_count==10)
					process_count=0;
				if(USB_check()){
					if(myusb.out_coming){
						myusb.out_coming = 0;
						USB_read_Raspberry();
					}
				}
				if(myusb.out_read_flag){
					myusb.out_read_flag = 0;
					myusb.rcv_timeout = TIMEOUT_MS;
					//refill per 8ms, -1 each ms in timer
					USB_read_Process();
				}								
			}//attitude compute end			
			idle_time++;
			//lock break
		}//flight loop end
	}//big loop end
}//main end

void init_loop(void)
{
	/*This function executes loops similar to the flight loop
	during which average values are got. After that, reference
	pressure, reference latitude longitude, and static acceleration 
	corrections in geographical coordinate are obtained*/
	#define LOOP_TIMES 300
	float sum_pressure = 0;
	int avgGlobAcc[3] = {0,0,0},sumGlobAcc[3] = {0,0,0},GlobAcc[3] = {0,0,0},avgBdyAcc[3] = {0,0,0};
	short baro_cnt = 0,acc_cnt = 0;
	short i = 0,j = 0;
#if OUTDOOR
#if WAIT_GPS
	while(gps.sat<6 || gps.lat == 0 || gps.lon == 0){
		if(gps.gpsflag){
			j = !j;
			if(j)
				beep(ON);
			else
				beep(OFF);
			gps.gpsflag = 0;
			get_gps_data();
		}		
	}
	while(!(gps.gpsflag == 1));
	gps.gpsflag = 0;
	get_gps_data();
	gps_pos_init();
	beep(OFF);
#endif	
#endif
	baro.temp_pres_switch = TEMP_SWITCH;
	while(i < LOOP_TIMES){//init loop
		if(smpl.Flag500Hz){
			smpl.Flag500Hz = 0;
			i++;			
			attitude_compute();
			get_geo_acc(GlobAcc);
			if(i > 200){
				for(j = 0; j < 3; j++)
					sumGlobAcc[j] += GlobAcc[j];
				acc_cnt++;
			}
			switch(process_count){
			case 0: case 2: case 4: case 6: case 8:
			
				if(baro.updated){
					if(baro.updated == D1_UPDATED){
						data2pressure();
						if(i > 100){
							sum_pressure += baro.pressure;
							baro_cnt++;
						}
						baro.temp_pres_switch = TEMP_SWITCH;
					}
					else if (baro.updated == D2_UPDATED){
						data2tempeture();
						baro.temp_pres_switch = PRES_SWITCH;
					}
					baro.updated = 0;
					
				}
				break;
				
			case 1: case 3: case 5: case 7: case 9:
				break;
			default:
				break;
			}
			process_count++;
			if(process_count==10)
				process_count=0;
		}	
	#if INDOOR
		if(vicon.xbeeflag){
			vicon.xbeeflag = 0;
			get_xbee_data();
		}
	#elif OUTDOOR
		if(gps.gpsflag){
			gps.gpsflag = 0;
			get_gps_data();
			gps_pos_corr(GPS_PERIOD);//smpl.UARTreceiveCount*1000/PIT_FREQ);
		}
	#endif
	}//end of i++ loops

#if INDOOR
	vicon_pos_init();
#elif OUTDOOR
	gps_pos_init();
#endif
	command_init();
	baro.refPressure = sum_pressure / baro_cnt;
	for(j = 0; j < 3; j++){
		avgGlobAcc[j] = sumGlobAcc[j] / acc_cnt;
	}
	glob2body(avgBdyAcc, avgGlobAcc, 3);
	corr_geo_acc(avgBdyAcc);
	process_count=0;
}
int self_check(void)
{
	/*Self check if sensor data is available, 
	and if they are reasonable. Returns 1 if good*/
	if(sens.az < 5000)
		return 1;
	else if(cmd.rc[2] > -850 || cmd.rc[2] < -1200)
		return 2;
	else if(baro.refPressure > 120000 || baro.refPressure < 80000)
		return 3;
//	else if(pos.x_est[0] / 1000 > 3000||pos.x_est[0] / 1000< -3000
//	||pos.y_est[0] / 1000 > 3000||pos.y_est[0] / 1000 < -3000
//	||pos.z_est[0] / 1000 > 1000||pos.z_est[0] / 1000 < -1000)
//		return 4;
	else
		return 0;
}
void ground_work_once(void)
{


}
void Process500Hz(void)
{
 //	beep(ON);
//	static short g_store[20],a_store[20];
//	static int ptr=0;
//	if(ptr==19){
//		ptr = 0;
//	}else{
//		ptr++;
//	}
//	g_store[ptr] = sens.gx;
//	a_store[ptr] = sens.az;
	
	attitude_compute();
	pos_predict(ATT_PERIOD,process_count);
	attitude_control(ATT_PERIOD);
	if(!mode.locked)
		put_motors();
	else
		motor_cut();
//	beep(OFF);
}
void Process250Hz_A(void)
{
	static unsigned int usb_count = 0;
	usb_count++;
	if(mode.FlightMode == POS_CTRL){
		position_control(POS_CTRL_PERIOD);
	}
	else if(mode.FlightMode == MANUEL || mode.FlightMode == ALT_CTRL){
		manual_R_sp_generate();
		altitude_control(POS_CTRL_PERIOD);
	}
	else if(mode.FlightMode == RASP_MANUEL || mode.FlightMode == RASP_ALT 
		|| mode.FlightMode == RASP_POS || mode.FlightMode == RASP_NURBS)
	{// in all raspPi modes, rasp gives q_sp and thrust force
		Qsp2Rsp();
		set_thrust_force();
	}
	if(usb_count > 1){//125Hz
		usb_count=0;
		if(myusb.connected){
			USB_write_Raspberry(1024,'s');
		}
	}
}
void Process250Hz_B(void)//baro, mag, 
{
	static unsigned int cps2read=0, baro_cnt=0;
	#if WAIT_GPS
	static unsigned int pos2corr=0;
	#endif
	#if INDOOR
	static unsigned int data_lost_times=0;
	#endif
	if(baro.updated){
		if(baro.updated == D1_UPDATED){
			data2pressure();
			if(baro.refPressure < 120000 && baro.refPressure > 80000){
				baro.alt=get_altitude();
				baro_pos_corr(14);
			}
		}
		else if (baro.updated == D2_UPDATED){
			data2tempeture();
		}
		baro.updated = 0;
		baro_cnt++;
		if(baro_cnt==10){
			baro_cnt = 0;
			baro.temp_pres_switch = TEMP_SWITCH;//for timer-spi's use
		}
		else{
			baro.temp_pres_switch = PRES_SWITCH;
		}
		cps2read = 1;//cps read next round
		return;
	}
	if(cps2read){
	#if PWM16
	#else
		continue_cps_read();
	#endif
		cps2read = 0;
	}
#if OUTDOOR
#if WAIT_GPS
	if(gps.gpsflag){
		gps.gpsflag = 0;	
		get_gps_data();
		pos2corr = 1;//leave the cpu load to next round
		return;
	}
	if(pos2corr){
		pos2corr = 0;
		gps_pos_corr(GPS_PERIOD);
	}
#else
	zero_pos_corr(4);

#endif
#elif INDOOR		
	if(vicon.xbeeflag){
		vicon.xbeeflag = 0;
		pos2corr = get_xbee_data();//get_xbee_data returns 1 when not lost
		if(pos2corr == 0)//data lost
			data_lost_times++;
		return;						
	}
	if(pos2corr){
		pos2corr = 0;
		vicon_pos_corr((1+data_lost_times)*VICON_PERIOD);
		data_lost_times = 0;
	}
#endif
}

void Process50Hz(void)
{
	static unsigned int adc_count=0,uart_count=0,led_count=0;
	get_rc(RADIO_PERIOD);
	if(mode.l_FlightMode != mode.FlightMode){						
		if(cmd.rc[2] < -819){
			if(mode.FlightMode == POS_CTRL){
			#if OUTDOOR
				gps_pos_init();			
			#endif			
			}
		}
		command_init();
	}
	if(cmd.rc[2] < -819 && mode.FlightMode == MANUEL){//-0.8
		reset_variables();
		//must not reset when changing mode, the int of att will continue to serve in pos_ctrl
	}
	if(mode.CalibrationMode != 0){
		motor_cut();
		led_ctrl(LED1,ON);
		led_ctrl(LED2,ON);
		if(mode.CalibrationMode != mode.l_CalMode){
			ground_work_once();//data storage etc.
		}	
	}
	else {		
	//0 off, 1 blink, 2 on
	//blue(front) 
	//two manuel modes 0; two alt modes 1; two pos modes 2
	//yellow(back) 
	//no usb, onboard modes 0; usb connect and offboard 1; 
	//usb connect but onboard 2
		led_count++;
		if(led_count>=10){
			led_count = 0;
			if(mode.FlightMode == POS_CTRL || mode.FlightMode == RASP_POS
				||mode.FlightMode == ALT_CTRL || mode.FlightMode == RASP_ALT) 
				led_ctrl(LED2,ON);
			else led_ctrl(LED2,OFF);
			if(myusb.rcv_timeout)//usb running
				led_ctrl(LED1,ON);
			else
				led_ctrl(LED1,OFF);
			if(!mode.locked)
				led_ctrl(LED3,ON);
		}
		else if(led_count==5){
			led_ctrl(LED3,OFF);
			if(mode.FlightMode == ALT_CTRL || mode.FlightMode == RASP_ALT)
				led_ctrl(LED2,OFF);
			if(mode.offboard)
				led_ctrl(LED1,OFF);
		}
	}
	
	adc_count++;
	if(adc_count>=50){//1Hz
		adc_count = 0;
		adc.battery = adc_get_converted();
		if(adc.battery < BAT_WARNING){
			beep(ON);
		}
		else beep(OFF);
		adc_start_conversion();	
	//	led_ctrl(LED3,OFF);
	}
	else if(adc_count==20){
		beep(OFF);
	//	led_ctrl(LED3,ON);
	}
	
	uart_count++;
	if(uart_count>=2){//>=5-10Hz
		uart_count = 0;
		data_select();
		idle_time = 0;
		if(cmd.data2send != sendNON){
			refill_tx(data2,18);
		}
	}
	#if PWM16
	pca_write();
	#endif
	if(cmd.SonarEnable)
		sonar_pos_corr(RADIO_PERIOD);
}
