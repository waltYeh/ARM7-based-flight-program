#ifndef FIXED_MATRIX_CONTROL_H
#define FIXED_MATRIX_CONTROL_H
void position_control(short dt);
void altitude_control(short dt);
void attitude_control(short dt);
void manual_R_sp_generate(void);
void Qsp2Rsp(void);
void set_thrust_force(void);
void reset_variables(void);


#endif
