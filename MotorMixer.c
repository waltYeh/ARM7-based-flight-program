#include "global.h"
#include "PWM.h"
#include "Pins.h"
#include "MotorMixer.h"
void force2output(int force[4], unsigned short duty[4], unsigned int battery)
//mNewton to 0~2400
{
	#if F450
		#define C 310
	#elif XINSONG
		#define C 410
	#elif F330
		#define C 300
	#elif F240
		#define C 200	
	#endif
	int k,i;
	if(battery == 0){
		k = 110;
	} else{
		//full4.20V(4.05V) k=100, low3.80V(3.65V) k=120
		k = constrain(303-(int)battery/20,95,125);
	}
	for(i=0;i<4;i++){
		duty[i] = force[i] * k / C + 2650;
	}
}
void put_motors(void)
{
	int motorForce[4] = {0,0,0,0};
	unsigned short motorDuty[4] = {2400,2400,2400,2400};
	short i;
	if(cmd.rc[2] < -850){
		motorForce[0] = 0;
		motorForce[1] = 0;
		motorForce[2] = 0;
		motorForce[3] = 0;
		for(i=0;i<4;i++){
			motorDuty[i]=2400;
		}
	}
	else{
	#if PLUS
/*			A								inv(A)
			[  1,  1, 1,  1]				[ 1/4, -1/(2*d),        0,  1/(4*c)]
			[ -d,  0, d,  0]				[ 1/4,        0, -1/(2*d), -1/(4*c)]
			[  0, -d, 0,  d]				[ 1/4,  1/(2*d),        0,  1/(4*c)]
			[  c, -c, c, -c]				[ 1/4,        0,  1/(2*d), -1/(4*c)]*/
		motorForce[0] = (output.thrustForce>>2) - output.pitchMmt / (ROTOR_DIST<<1) + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[1] = (output.thrustForce>>2) - output.rollMmt / (ROTOR_DIST<<1) - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[2] = (output.thrustForce>>2) + output.pitchMmt / (ROTOR_DIST<<1) + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[3] = (output.thrustForce>>2) + output.rollMmt / (ROTOR_DIST<<1) - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
	#elif CROSS
/*			A													inv(A)
			[        1,        1,        1,       1]			[ 1/4, -sqrt2/(4*d),  sqrt2/(4*d),  1/(4*c)]
			[ -d/sqrt2, -d/sqrt2,  d/sqrt2, d/sqrt2]			[ 1/4, -sqrt2/(4*d), -sqrt2/(4*d), -1/(4*c)]
			[  d/sqrt2, -d/sqrt2, -d/sqrt2, d/sqrt2]			[ 1/4,  sqrt2/(4*d), -sqrt2/(4*d),  1/(4*c)]
			[        c,       -c,        c,      -c]			[ 1/4,  sqrt2/(4*d),  sqrt2/(4*d), -1/(4*c)]
		d=D/2
		torq=c*f*/
		motorForce[0] = (output.thrustForce>>2) + (-output.pitchMmt + output.rollMmt) / D2_SQRT2 + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[1] = (output.thrustForce>>2) + (-output.pitchMmt - output.rollMmt) / D2_SQRT2 - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[2] = (output.thrustForce>>2) + (output.pitchMmt - output.rollMmt) / D2_SQRT2 + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[3] = (output.thrustForce>>2) + (output.pitchMmt + output.rollMmt) / D2_SQRT2 - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
	#endif		
		force2output(motorForce, motorDuty, adc.battery);
	}	
	//channel-PWMlinesign order- 1324
	pwm_set_duty_cycle(0, constrain(motorDuty[0],2400,4790));
	pwm_set_duty_cycle(1, constrain(motorDuty[1],2400,4790));
	pwm_set_duty_cycle(2, constrain(motorDuty[2],2400,4790));
	pwm_set_duty_cycle(3, constrain(motorDuty[3],2400,4790));
}
