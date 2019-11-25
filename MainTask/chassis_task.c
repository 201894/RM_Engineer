/** @file chassis_task.c
 *  @version 1.0
 *  @date Nev 2019
 *
 *  @brief chassis control task
 *
 *
 */


#include "info_handle_task.h"
#include "chassis_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "ramp.h"
#include "math.h"
#include "string.h"
#include "cmsis_os.h"
#include "STMGood.h"

#define pid_chassis_adjust         1

void chassis_task(void const *argu)
{
	 portTickType ChassisTaskWakeTime;
   for(;;)
   {
		ChassisTaskWakeTime = xTaskGetTickCount();
    taskENTER_CRITICAL(); 		 
		if(chassis.mode)
		{

			chassis_link_handle();			
			for(int i=0;i<6;i++)
			{
				pid_ast(&pid_chassis[i],chassis.target[i],moto_chassis[i].speed_rpm);
				chassis.current [i] = (int16_t)pid_chassis[i].ctrOut;	
			}		
			if(chassis.mode==AUTO_MODE)
			{
			 send_chassis_cur(0x200,chassis.current[0],chassis.current[1],0,0);
			}
      else
			{
			 send_chassis_cur(0x200,chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);			
			}
			if (chassis.vw <200 | chassis.vy <200)
				send_chassis_cur(0x1ff,chassis.current[4],chassis.current[5],0,0);	
			else
				send_chassis_cur(0x1ff,0,0,0,0);				
		}
		else
		{
			send_chassis_cur(0x200,0,0,0,0);
			send_chassis_cur(0x1ff,0,0,0,0);			
		}
		taskEXIT_CRITICAL();
    osDelayUntil(&ChassisTaskWakeTime,5);	
   }
}

void mecanum_algorithm(float vx,float vy, float vw,int16_t speed[])
{
    static float Buffer[6];
    Buffer[0] = vx + vy + vw + chassis.UpStairVx;
    Buffer[1] = vx - vy - vw + chassis.UpStairVx;
    Buffer[2] = vx - vy + vw + chassis.UpStairVx;
    Buffer[3] = vx + vy - vw + chassis.UpStairVx;	
    Buffer[4] = vx + vw + chassis.UpStairVx + chassis.ExtraVx; 
    Buffer[5] = vx - vw + chassis.UpStairVx + chassis.ExtraVx;
	
	  speed[0] =  Buffer[0];
    speed[1] = -Buffer[1];
    speed[2] =  Buffer[2];
    speed[3] = -Buffer[3];
	  if(chassis.mode == UPSTAIR_MODE)
		{
			speed[4] =  1.3*Buffer[4];
			speed[5] = -1.3*Buffer[5];	
		}		
		else
		{
			speed[4] =  Buffer[4];
			speed[5] = -Buffer[5];				
		}	
}

void gyro_algorithm(void)
{
    /* the angle which is calculating by using the atan2 function between Vx and Vy */
	static float atan_angle = 0.0f;
	/* angle diff between gimbal and chassis */
	static float diff_angle = 0.0f;
	/* the merge spd by sqrt(Vx^2 + Vy^2) */
	static float merge_spd = 0.0f;
	merge_spd = sqrt(chassis.vx * chassis.vx + chassis.vy * chassis.vy);
	atan_angle = atan2(chassis.vy, chassis.vx);
	atan_angle *= 180.0f/3.1415926f;
	diff_angle = /*yaw.BackAngle - */-(chassis.target_angle); 
	diff_angle += atan_angle;	
	/* calcualate the atan_angle every time */
	diff_angle = diff_angle - (int)(diff_angle / 360.0f) * 360.0f;
	//printf("diff_angle = %f\r\n", diff_angle);
	diff_angle *= 3.1415926f/180.0f;
	chassis.vx = merge_spd * cosf(-diff_angle);
	chassis.vy = -merge_spd * sinf(-diff_angle);	
	mecanum_algorithm(chassis.vx,chassis.vy,chassis.vw,chassis.target);	
}
void chassis_link_handle(void)
{
	#if  pid_chassis_adjust
	  pid_adjust(&pid_link_out,12,0,0);
	  pid_adjust(&pid_link_in,15,0,0);
	  pid_link_out.errILim = 400;
	  pid_link_out.MaxOut = 1000;
	  pid_link_in.errILim = 5000;	
	  pid_link_in.MaxOut   = 20000;  	  
	#endif	
	
	  pid_ast(&pid_link_out,chassis.target_angle,gyro_yaw.angle_z); //lose the constant target		  		  // round 0-1  21  43   
	  chassis.link_buffer = pid_link_out.ctrOut;
	  pid_ast(&pid_link_in,chassis.link_buffer,gyro_yaw.yaw_speed);			
	 // chassis.vw = -pid_link_in.ctrOut; 	
	  mecanum_algorithm(chassis.vx,chassis.vy,chassis.vw,chassis.target);	
}

void chassis_param_init(void)
{
	for(int i=0;i<6;i++)
	{
	  memset(&moto_chassis[i],0,sizeof(moto_param));
	  PID_struct_init(&pid_chassis[i],chassis_errILim,chassis_maxOut,chassis_kp,chassis_ki,chassis_kd);              //          
	}
	memset(&chassis,0,sizeof(chassis_t));	
	PID_struct_init(&pid_link_out,link_out_errILim,link_out_maxOut,link_out_kp,link_out_ki,link_out_kd); 
	PID_struct_init(&pid_link_in,link_in_errILim,link_in_maxOut,link_in_kp,link_in_ki,link_in_kd); 	
}
 


