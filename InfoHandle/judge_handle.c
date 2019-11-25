
/** @file judge_handle.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief deal with the judge_system and turn the info into shoot_flag
 *
 */
 
#include "judge_handle.h"
#include <string.h>
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_can.h"

//#include "detect_task.h"
int heat_remain=0,bcurheat=0,last_heat=0;
int heat_flag = 0;
extern int Bullet_Number;
int shoot_number=0;
int number_flag;

/*	                        /s                          /s
level     17mm      cd           42mm    cd
  1         240         40            200       20  
  2         360         60            300       40
  3	     480         80            400       60 
ext_power_heat_data.shooter_heat    50HZ  
*/	
int increment;
int16_t local_update;
extern int mm42heat_debug;
void heat_local_update(uint8_t level)
{
  local_update++;
  if (local_update==2)
  {
	switch (level)
	{
		case 0:
		{
		 if(mm42heat_debug <= 1) mm42heat_debug = 0;
			else mm42heat_debug -= 2;		
		}break;		
		case 1:
		{
		 if(mm42heat_debug <= 1) mm42heat_debug = 0;
			else mm42heat_debug -= 2;		
		}break;
		case 2:
		{
		 if(mm42heat_debug <= 1) mm42heat_debug = 0;
			else mm42heat_debug -=4;			
		}break;	
		case 3:
		{
		 if(mm42heat_debug <= 1) mm42heat_debug = 0;
			else mm42heat_debug -= 6;				
		}break;
		default:
		{
		 if(mm42heat_debug <= 1) mm42heat_debug = 0;
			else mm42heat_debug -= 2;				
		}break;				
	 }
       local_update = 0;	
  }
}

int mm42heat_ctrl(uint16_t curheat,uint8_t level)
{
	 last_heat = bcurheat;	
	 bcurheat =(int) curheat;	
	if (bcurheat - last_heat>50)
	{
		Bullet_Number--;
		shoot_number++;
	}
    if(shoot_number)
	   number_flag = 1;
	if (!Bullet_Number)
		shoot_number =0;
	if (Bullet_Number<0)
	    Bullet_Number = 0;
	switch (level)
	{
		case 1:
		{
		  heat_remain = 197- bcurheat;
		}break;
		case 2:
		{
		  heat_remain = 300- bcurheat;				
		}break;	
		case 3:
		{
		  heat_remain = 400- bcurheat;				
		}break;		
        default:
		{
		  heat_remain = 200- bcurheat;				
		}break;	
	}
	if(heat_remain>100)
       return 1;	
	else
		return 0;
}

void judge_global_init(void)
{
  	memset(&ext_game_state,0,sizeof(ext_game_state_t));
   	memset(&ext_power_heat_data,0,sizeof(ext_power_heat_data_t));
  	memset(&ext_game_robot_state,0,sizeof(ext_game_robot_state_t));
  	memset(&ext_buff_musk,0,sizeof(ext_buff_musk_t));
  	memset(&ext_shoot_data,0,sizeof(ext_shoot_data_t));    
	memset(&ext_robot_hurt,0,sizeof(ext_robot_hurt_t)); 
	ext_game_robot_state.robot_level = 1;
    ext_power_heat_data.shooter_heat0 = 0;
    ext_power_heat_data.shooter_heat1 = 0;	
}
