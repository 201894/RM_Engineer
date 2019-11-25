
/** @file info_get_task.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief get infantry sensor and control information
 *
 *
 */
#include "info_handle_task.h"
#include "STMGood.h"	
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "ano_dt.h"
#include "bsp_io.h"
#include <math.h>

chassis_t      chassis;

void info_handle_task(void const * argument)
{
   for(;;)
   {
		  rc_target_handle(&rc);
		  can_debug();
		  get_main_ctrl_mode();
	    osDelay(5);
   }

}

void get_moto_info(void)
{
	// 	chassis_moto_info :
   for (uint8_t i = 0; i < 6; i++)
   {
     chassis.wheel_speed[i] = moto_chassis[i].speed_rpm;
   }  
}

void rc_target_handle(rc_info_t *rc)
{	
    chassis.vx = (rc->ch1)*MOVENORMAL;	
	  chassis.vy = (rc->ch3)*MOVENORMAL; 
	  chassis.vw = (rc->ch2)*MOVENORMAL*1.2f;        
    (fabs(rc->ch2)>=10) ? \
	     (chassis.target_angle -= (float)(rc->ch2*RC_YAW_RATIO)) : NULL;	
}

void get_main_ctrl_mode(void)
{

   switch(rc.sw1)
	 {
	 	 case RC_UP:
		 {
			chassis.mode = SAFETY_MODE;		
		 	switch(rc.sw2)
			{
			    case RC_UP:
			    {
	   
			    }
			    break;
			    case RC_MI:
			    {			
				
			    }
			    break;
		      case RC_DN:
			    {
			
		 	    }			
			    break;
			    default:
			    {
			    }
			    break;											
			 }

		 }	break;			 
	 	 case RC_MI:
		 {
			chassis.mode = NORMAL_MODE;				 
		 	switch(rc.sw2)
			{
			    case RC_UP:
			    {
						chassis.mode = AUTO_MODE;	         
			    }
			    break;
			    case RC_MI:
			    {
    
			    }
			    break;
		      case RC_DN:
			    {
						chassis.UpStairVx = 0;
            chassis.ExtraVx = 0;		
			    }
			    break;
			    default:
			    {					
			    }
			    break;											
			 }
			
		 }break;		 
	 	 case RC_DN:
		 {
			 chassis.UpStairVx = 0; 
       chassis.ExtraVx = 0;					 
		 	switch(rc.sw2)
			{
			    case RC_UP:
			    {
    				chassis.mode	 = UPSTAIR_MODE;		  							 
			    }
			    break;  
			    case RC_MI: 
			    { 
						chassis.mode = NORMAL_MODE; 						
			    } 
			    break; 
		      case RC_DN:
			    {
						chassis.mode = SAFETY_MODE;
			    } 
			    break; 
			    default:
			    {		
						chassis.mode = SAFETY_MODE;						
			    }
			    break;											
			 }		 
		 }	break;	
	     default:
	     {
		   } break;				 
	 } 
}

uint32_t can_cnt[10];
uint32_t identity_time_last[10],cnt_cnt;
int identity_time_ms[10];
void can_debug(void)
{
	if (cnt_cnt==0)
	{
		for(int i=0;i<10;i++)
		{
	      identity_time_last[i] =  can_cnt[i];
		}
	}
	   cnt_cnt++;
	if (cnt_cnt ==200)
	{
	    cnt_cnt = 0;
		for(int i=0;i<=10;i++)
		{
		   identity_time_ms[i] =  can_cnt[i] - identity_time_last[i];
		}
	}
}
void moto_info_init(void)
{ 
	rc_init(); 	
	memset(&chassis,0,sizeof(chassis_t));  

}













