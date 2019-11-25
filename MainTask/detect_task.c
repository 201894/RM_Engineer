
/** @file detect_task.c
 *  @version 4.0
 *  @date JUNE 2019
 *
 *  @brief detect module offline or online task
 *
 *
 1. 电机堵转检测
 2. PC信息交互
 3. 用户界面
 */

#include "detect_task.h"
#include "bsp_uart.h"
#include "Vision_decode.h"
#include "cmsis_os.h"
#include "km_handle.h"
#include "pid.h"
#include "info_handle_task.h"
#include "bsp_io.h"
#include "freertos.h"
#include "tim.h"
#include "bsp_can.h"
#include "math.h"
extern int Bullet_Number,speed_flag;
UBaseType_t detect_stack_surplus;
int mc_speed_flag;
/* detect task global parameter */
global_err_t g_err;
judge_send_t judge_send;
/* detect task static parameter */
static offline_dev_t offline_dev[STIR_M3_OFFLINE + 1];
/**
  * @brief     initialize detector error_list
  * @usage     used before detect loop in detect_task() function
  */
void detector_init(void)
{
	  g_err.err_now = NULL;
	  g_err.list[BOTTOM_DEVICE].dev    = NULL;
	  g_err.list[BOTTOM_DEVICE].enable = 0;

	  /* initialize device error type and offline timeout value */
	  for (uint8_t i = PIT_GYRO_OFFLINE; i < ERROR_LIST_LENGTH; i++)
	  {
		  
		if (i <= STIR_M3_OFFLINE)
		{
		  offline_dev[i].set_timeout = 200; //ms
		  offline_dev[i].last_time   = 0;
		  offline_dev[i].delta_time  = 0;
		  
		  g_err.list[i].dev  = &offline_dev[i];
		  g_err.list[i].type = DEV_OFFLINE;
		}
		else if (i == BULLET_JAM)
		{
		  g_err.list[i].dev  = NULL;
		  g_err.list[i].type = DEV_RUNNING_ERR;
		}
		else if (i <= GIMBAL_CONFIG_ERR)
		{
		  g_err.list[i].dev  = NULL;
		  g_err.list[i].type = SYS_CONFIG_ERR;
		}
	  }
		
	  /* initialize device error detect priority and enable byte */
	  
	  g_err.list[REMOTE_CTRL_OFFLINE].err_exist = 0;
	  g_err.list[REMOTE_CTRL_OFFLINE].pri       = 14;  //max priority
	  g_err.list[REMOTE_CTRL_OFFLINE].enable    = 1;

	  g_err.list[JUDGE_SYS_OFFLINE].err_exist  = 0;
	  g_err.list[JUDGE_SYS_OFFLINE].pri        = 13;
	  g_err.list[JUDGE_SYS_OFFLINE].enable     = 1;

	 g_err.list[PC_SYS_OFFLINE].err_exist = 0;
	 g_err.list[PC_SYS_OFFLINE].pri       = 12;
	 g_err.list[PC_SYS_OFFLINE].enable    = 1;
	  
	  g_err.list[YAW_GYRO_OFFLINE].err_exist  = 0;
	  g_err.list[YAW_GYRO_OFFLINE].pri        = 11;
	  g_err.list[YAW_GYRO_OFFLINE].enable     = 1;
	  
	  g_err.list[PIT_GYRO_OFFLINE].err_exist    = 0;
	  g_err.list[PIT_GYRO_OFFLINE].pri          = 10;
	  g_err.list[PIT_GYRO_OFFLINE].enable       = 1;
	  
	  g_err.list[GIMBAL_YAW_OFFLINE].err_exist = 0;
	  g_err.list[GIMBAL_YAW_OFFLINE].pri       = 9;
	  g_err.list[GIMBAL_YAW_OFFLINE].enable    = 1;
	  
	  g_err.list[GIMBAL_BPIT_OFFLINE].err_exist = 0;
	  g_err.list[GIMBAL_BPIT_OFFLINE].pri       = 9;
	  g_err.list[GIMBAL_BPIT_OFFLINE].enable    = 1;
	  
	  g_err.list[STIR_M1_OFFLINE].err_exist = 0;
	  g_err.list[STIR_M1_OFFLINE].pri        = 7;
	  g_err.list[STIR_M1_OFFLINE].enable    = 1;
	  
	  g_err.list[STIR_M2_OFFLINE].err_exist = 0;
	  g_err.list[STIR_M2_OFFLINE].pri        = 7;
	  g_err.list[STIR_M2_OFFLINE].enable    = 1;
	  
	  g_err.list[STIR_M3_OFFLINE].err_exist = 0;	  
	  g_err.list[STIR_M3_OFFLINE].pri        = 7;
	  g_err.list[STIR_M3_OFFLINE].enable    = 1;	  
	  
	  g_err.list[CHASSIS_M1_OFFLINE].err_exist = 0;
	  g_err.list[CHASSIS_M1_OFFLINE].pri       = 6; 
	  g_err.list[CHASSIS_M1_OFFLINE].enable    = 1;
	  
}  
/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_detector_hook(int err_id)
{
  if (g_err.list[err_id].enable)
      g_err.list[err_id].dev->last_time = HAL_GetTick();
}

void detector_param_init(void)
{
  detector_init();
  g_err.beep_tune = DEFAULT_TUNE;
  g_err.beep_ctrl = 0;  
  led_init;
}

 /**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
uint32_t detect_time_last;
int detect_time_ms;
int  vision_cnt;
int heat_cnt,lka_cnt,mm17_flag,heat_update_flag;
extern float  Z;
extern uint8_t bullet_flag;
uint8_t maskflag;
int stir_stall_cnt,stir_stall_cnt1;
extern int16_t stir_speed;
extern int mm42heat_debug;
uint8_t TxData1[8];
void detect_task(void const *argu)
{
  LED_R_OFF;
  LED_G_OFF;
  uint32_t detect_wake_time = osKernelSysTick();
  while(1)
  {
//	  Light_Detection(Light_Sensor);
//	  flow_led();
	  can_debug();
      detect_time_ms = HAL_GetTick() - detect_time_last;
      detect_time_last = HAL_GetTick();
 
    if (!g_err.list[JUDGE_SYS_OFFLINE].err_exist)
	{
	  heat_cnt++;
	  if (heat_cnt>=8)
	  {
		 mm42heat_debug =  ext_power_heat_data.shooter_heat1;
		 heat_cnt = 0;
	  }
  }
	  motor_stall_detect();
	  pc_data_ex(ext_game_robot_state.robot_id,Bullet_Number,4);  //  pc信息交互 
	  heat_local_update(ext_game_robot_state.robot_level); // 42mm本地热量更新	  
	  judge_light_detect();                     
	  judge_send.data[0] = (float)Bullet_Number;
	  judge_send.data[1] =  Z;
	  judge_send.mask = judge_send.light_ctrl.bit0| \
									judge_send.light_ctrl.bit1<<1| \
									judge_send.light_ctrl.bit2<<2| \
									judge_send.light_ctrl.bit3<<3| \
									judge_send.light_ctrl.bit4<<4| \
									judge_send.light_ctrl.bit5<<5;	  
 	 judge_info_send(judge_send.data[0],judge_send.data[1],judge_send.mask);	  
	  /* module offline detect */
    module_offline_detect();
    if (g_err.err_now != NULL)
    {
      LED_R_OFF;
      module_offline_callback();
    }
    else
    {
      g_err.beep_ctrl = 0;
      LED_G_ON;
    }    
    BEEP_TUNE = g_err.beep_tune;
    BEEP_CTRL = g_err.beep_ctrl;//g_err.beep_ctrl;
    
   // detect_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    osDelayUntil(&detect_wake_time, 50);
  }
}
  
static void module_offline_detect(void)
{
  int max_priority = 0;
  int err_cnt      = 0;
  for (uint8_t id = PIT_GYRO_OFFLINE; id <= STIR_M3_OFFLINE; id++)
  {
    g_err.list[id].dev->delta_time = HAL_GetTick() - g_err.list[id].dev->last_time;
    if (g_err.list[id].enable 
        && (g_err.list[id].dev->delta_time > g_err.list[id].dev->set_timeout))
    {
      g_err.list[id].err_exist = 1; //this module is offline
      err_cnt++;
      if (g_err.list[id].pri > max_priority)
      {
        max_priority     = g_err.list[id].pri;
        g_err.err_now    = &(g_err.list[id]);
        g_err.err_now_id = (err_id_e)id;		  
      }
    }
    else
    {
      g_err.list[id].err_exist = 0;
    }
  }

  if (!err_cnt)
  {
    g_err.err_now    = NULL;
    g_err.err_now_id = BOTTOM_DEVICE;
  }
}

static void module_offline_callback(void)
{
	
    g_err.err_count++;
    if (g_err.err_count > 40)
    g_err.err_count = 0;

  switch (g_err.err_now_id)
  {
    case REMOTE_CTRL_OFFLINE:
    case GIMBAL_YAW_OFFLINE:
    case STIR_M1_OFFLINE:    			
    {
      if (g_err.err_count == 1)
      {
        LED_R_ON;       
		 __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
      }
      else
      {
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        LED_R_OFF;
      }
    }break;
    

	case JUDGE_SYS_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
		 __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }
      else
      {
        LED_R_OFF;
		__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,0);	
        g_err.beep_ctrl = 0;
      }
    }break;
	
    case GIMBAL_BPIT_OFFLINE:		
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
		  __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }
      else
      {
        LED_R_OFF;
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        g_err.beep_ctrl = 0;
      }
    }break;  
	
    case STIR_M2_OFFLINE:   	
    case STIR_M3_OFFLINE:   	
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
		  __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }
      else
      {
        LED_R_OFF;
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        g_err.beep_ctrl = 0;
      }
    }break;  
    case PC_SYS_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19
          || g_err.err_count == 25)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
		 __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }	  
      else
      {
        LED_R_OFF;
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        g_err.beep_ctrl = 0;
      }
    }break;   
	case PIT_GYRO_OFFLINE:
	case YAW_GYRO_OFFLINE:	
    case CHASSIS_M1_OFFLINE:		
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7
          || g_err.err_count == 13
          || g_err.err_count == 19
          || g_err.err_count == 25
	      || g_err.err_count == 31)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
		 __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }	  
      else
      {
        LED_R_OFF;
		__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        g_err.beep_ctrl = 0;
      }
    }break;    	
    default:
    {
      LED_R_ON;
	  __HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
      g_err.beep_ctrl = 0;
    }break;
  }
}

extern uint32_t can_cnt[10];
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

	if (cnt_cnt ==20)
	{
	    cnt_cnt = 0;
		for(int i=0;i<=10;i++)
		{
		   identity_time_ms[i] =  can_cnt[i] - identity_time_last[i];
		}
	}
}

/*
Light 1 :  Friction on/off
Light 2 :  basket  on/off
Light 3 :  supercap on/off
Light 4:   Friction on/off
Light 5 :  Friction on/off
Light 6 :  Friction on/off
*/

int protect_flag,protect_flag1,stall_flag;
void motor_stall_detect(void)
{
	
   // BIG STIR MOTOR
   if (fabs(pid_bullet_out.errNow)>=10)
   {
       stir_stall_cnt++;
	 if (stir_stall_cnt>=15&&protect_flag)
	 {
		 kernel_bullet.target -=72;
	     protect_flag = 0;
	     stir_stall_cnt = 0;
         stall_flag	= 1;	 
	 }
   }
   else
   {
	   stir_stall_cnt = 0;	   
	   protect_flag = 1;
   }
   
   if (fabs(pid_stir_out.errNow)>=10)
   {
       stir_stall_cnt1++;
	 if (stir_stall_cnt1>=20&&protect_flag1)
	 {
		 kernel_stir.target -=72;
	     protect_flag1 = 0;
		 
	     stir_stall_cnt1 = 0;	   		 
	 }
   }
   else
   {
	   stir_stall_cnt1 = 0;	   
	   protect_flag1 = 1;
   }   
}

/*
 bit0~~5, 1 GREEN  0 RED
*/
int bullet_flag_cnt;
int basket_flag;
extern int16_t fric_speed;
void judge_light_detect(void)
{
      if (abs(fric_speed)>=3000&&abs(pid_fric[0].errNow)<=100)
		 judge_send.light_ctrl.bit0 = 1;
	  else
		  judge_send.light_ctrl.bit0 = 0;
      if (abs(fric_speed)>=3000&&abs(pid_fric[1].errNow)<=100)
		 judge_send.light_ctrl.bit1 = 1;
	  else
		  judge_send.light_ctrl.bit1 = 0; 
	  
      if (abs(fric_speed)>=3600&&abs(pid_fric[0].errNow)<=100&&abs(pid_fric[1].errNow)<=100)
		 mc_speed_flag = 1; 
	  else
		  mc_speed_flag = 0; 
	  
	     if (!km.middle_flag)
		 {
	        judge_send.light_ctrl.bit2 = 1;
		    judge_send.light_ctrl.bit3 = 1;  	  
		 }
         else
		 {
	        judge_send.light_ctrl.bit2 = 0;
		    judge_send.light_ctrl.bit3 = 0;  	  
		 }
		 
}

/*
data1:  bullet number
data2:  capacity voltage
data3:  to be~~
mask:  friction wheel's state
*/
uint16_t judge_cnt;
void judge_info_send(float data1,float data2,uint8_t mask)
{
	judge_cnt++;
	if (judge_cnt==4)
	{
	  judge_cnt = 0;
	  data4bytes.f = data1;
	  TxData1[0] = data4bytes.c[0];
	  TxData1[1] = data4bytes.c[1];
	  TxData1[2] = data4bytes.c[2];
	  TxData1[3] = data4bytes.c[3];
	  data4bytes.f = data2;
	  TxData1[4] = data4bytes.c[0];
	  TxData1[5] = data4bytes.c[1];
	  TxData1[6] = data4bytes.c[2];
	  TxData1[7] = data4bytes.c[3];	
	  send_chassis_ms(0x300,TxData1);
	  TxData1[0] = mask;	
	  memset(&TxData1[1],0,7);
	  send_chassis_ms(0x302,TxData1);  
	}
}	

