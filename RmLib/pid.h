/** @file pid.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief pid parameter initialization, position and delta pid calculate
 *
 */	
#ifndef __pid_H__
#define __pid_H__

#include "stm32f4xx_hal.h"
typedef struct{
	float kp;
	float ki;
	float kd;	
	float errILim;
	float errNow; 
	float ctrOut;
	float errOld;
	float errP;
	float errI;
	float errD;
	float MaxOut;	
	float errIPoint;
	float errDPoint;	
}PID_Typedef;

typedef  struct{
		float kp;
		float ki;
		float kd;	
		float errNow;
		float dCtrOut;
		float ctrOut;	
		float errold1;
		float errold2;	
		float MaxOut;
}PID_IncrementType;

void PID_struct_init(
		PID_Typedef *pid,
		uint32_t errlLim,
		uint32_t maxout,

		float kp,
		float ki,
		float kd);
	
void pid_ast(PID_Typedef *pid,float target,float input);
void PID_IncrementMode(PID_IncrementType *pid);
void pid_adjust(
		PID_Typedef *pid,
		float kp,
		float ki,
		float kd);
extern  PID_Typedef pid_yaw_out;
extern  PID_Typedef pid_yaw_in;
extern  PID_Typedef pid_chassis[6];
extern	PID_Typedef pid_link;//use total_angle and constant to make pid
extern	PID_Typedef pid_link_out;// total_angle of 6020 supposed to be out_pid; Gyro.gz to handle the in_pid
extern	PID_Typedef pid_link_in;	
extern	PID_Typedef pid_ramp_out;// total_angle of 6020 supposed to be out_pid; Gyro.gz to handle the in_pid
extern	PID_Typedef pid_ramp_in;			
extern   PID_Typedef pid_pit_out;//6o2o
extern   PID_Typedef pid_pit_in;
extern   PID_Typedef pid_Rpit_out;//6020
extern   PID_Typedef pid_Rpit_in;
extern PID_Typedef IMG_PIT_OUT;
extern PID_Typedef IMG_YAW_OUT;
extern PID_Typedef IMG_RPIT_OUT;
extern	PID_Typedef pid_bullet_out;//M2006 x 2
extern	PID_Typedef pid_bullet_in;
extern	PID_Typedef pid_stir_out;//M2006 x 2
extern	PID_Typedef pid_stir_in;
extern	PID_Typedef pid_fric[2];

#endif
