
/** @file km_handle.c
 *  @version 2.0
 *  @date 3.29 2019
 *
 *  @brief get rc information and handle it
 *
 *
 */
 
 
#include "gimbal_task.h"
#include "ramp.h"
#include "km_handle.h"
#define VX_SPEED   660
#define VY_SPEED    660
#define SVX_SPEED   1200
#define SVY_SPEED    1200
/*
Target:
 1.  WASD                   chassis move 
   1.1 normal move 
   1.2 add capacity        shift 
 2. mouse.x  mouse.y   control yaw and pit 
 3. mouse.l   mouse.r   control shoot or not  
 4. R                            control get bullets 
 5. X                             Turn_mode 
 6.                               if use auto mode
*/

float chassis_para;
kb_ctrl_t km;
int speed_flag,rotate_flag;
ramp_t ramp_x ;
ramp_t ramp_y ;
extern int CAP1,deputy_capflag;
extern float  MaxOutSum,MinOutSum;
extern bool link_flag;

// 防抖处理
static int key_hook(uint16_t key){
	return (rc.kb.key_code  == key) && !(src.kb.key_code == key);
} 


// 长摁 有效
static void chassis_direction_ctrl(
	uint8_t shift,
 	uint8_t ctrl,   
	uint8_t up,                                            
	uint8_t down,	                                              
	uint8_t left,  
	uint8_t right)
{
	if(shift)
	{
	  if(capvot>=18.0f)
	  {		  
       if (up)
	    km.vx  =  SVX_SPEED*ramp_calc(&ramp_x);
       else if (down)
	    km.vx =  -SVX_SPEED*ramp_calc(&ramp_x);
	   else
	   {
	     km.vx  = 0;
	     ramp_init(&ramp_x,200);			   
	   }
       if (left)
	    km.vy =  -SVY_SPEED;
       else if (right)
	    km.vy =  SVY_SPEED;
	   else
	   {
	     km.vy  = 0;
	     ramp_init(&ramp_y,200);
	   }
     }
	  else
	  {
          if (up)
	        km.vx = 0.7*VX_SPEED*ramp_calc(&ramp_x);
          else if (down)
	        km.vx = -0.7*VX_SPEED*ramp_calc(&ramp_x);
	      else
          {
		    km.vx = 0;
	        ramp_init(&ramp_x,80);
	      }
         if (left)
	       km.vy =  -0.6*VY_SPEED*ramp_calc(&ramp_x);
         else if (right)
	       km.vy =  0.6*VY_SPEED*ramp_calc(&ramp_x);
	      else
          {
	        km.vy = 0;
	        ramp_init(&ramp_y,80); 	 	  
	      }			     
	  }
	}
	else if(ctrl)
	{
         if (up)
	       km.vx =  0.45*VX_SPEED*ramp_calc(&ramp_x);
         else if (down)
	       km.vx =  -0.45*VX_SPEED*ramp_calc(&ramp_x);
	     else
         {
		   km.vx = 0;
	       ramp_init(&ramp_x,80);
	     }
        if (left)
	      km.vy =  -0.5*VY_SPEED*ramp_calc(&ramp_y);
        else if (right)
	      km.vy =  0.5*VY_SPEED*ramp_calc(&ramp_y);
	     else
         {
	       km.vy = 0;
	        ramp_init(&ramp_y,80); 	 	  
	     }	
	}
	else
	{
	 if (CAP1==0)
	 {
       if (up)
	     km.vx = VX_SPEED*0.85*ramp_calc(&ramp_x);
       else if (down)
	     km.vx = -VX_SPEED*0.85*ramp_calc(&ramp_x);
	   else
       {
		 km.vx = 0;
	     ramp_init(&ramp_x,40);
	   }
       if (left)
	    km.vy = -VY_SPEED*0.65*ramp_calc(&ramp_y);
       else if (right)
	    km.vy = VY_SPEED*0.65*ramp_calc(&ramp_y);
	   else
       {
	     km.vy = 0;
	     ramp_init(&ramp_y,40); 	 	  
	   }
     }
		else
		{
          if (up)
	        km.vx = 0.7*VX_SPEED*ramp_calc(&ramp_x);
          else if (down)
	        km.vx = -0.7*VX_SPEED*ramp_calc(&ramp_x);
	      else
          {
		    km.vx = 0;
	        ramp_init(&ramp_x,80);
	      }
         if (left)
	       km.vy =  -0.6*VY_SPEED*ramp_calc(&ramp_y);
         else if (right)
	       km.vy =  0.6*VY_SPEED*ramp_calc(&ramp_y);
	      else
          {
	        km.vy = 0;
	        ramp_init(&ramp_y,80); 	 	  
	      }		
		}			
    }
}
// 点动 切换 模式
static void chassis_rotate_ctrl(uint16_t key)
{
    if (key_hook(key))
    km.rotate_flag = ! km.rotate_flag;
}
static void vision_rotate_ctrl(uint16_t key)
{
    if (key_hook(key))
    rotate_flag = ! rotate_flag;
}
static void ramp_mode_ctrl(uint16_t key)
{
    if (key_hook(key))
    km.ramp_flag = ! km.ramp_flag;
}
// 长摁取弹 松手反弹
static void chassis_middle_ctrl(uint16_t key)
{
    if (key_hook(key))
	{
       km.middle_flag = ! km.middle_flag;
       gimbal_yaw.target -=180;
	}
}


// 鼠标短摁 / 长摁
void key_fsm(kb_state_e *sta, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)		  
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
        if (sta == &km.lk_sta)
          km.lk_cnt = 0;
        else if(sta == &km.rk_sta)
          km.rk_cnt = 0;
        else if(sta == &km.wk_sta)
          km.wk_cnt = 0;	
        else if(sta == &km.ak_sta)
          km.ak_cnt = 0;	
        else if(sta == &km.sk_sta)
          km.sk_cnt = 0;		
        else if(sta == &km.dk_sta)
          km.dk_cnt = 0;			
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
        if (sta == &km.lk_sta)
        {
          if (km.lk_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.rk_sta)
        {
          if (km.rk_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.wk_sta)
        {
          if (km.wk_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.ak_sta)
        {
          if (km.ak_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.sk_sta)
        {
          if (km.sk_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }	
        else if(sta == &km.dk_sta)
        {
          if (km.dk_cnt++ > 500)
            *sta = KEY_PRESS_LONG;
        }		
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
	
    default:
    break;      
  }
}

void km_global_ctrl(void)
{
	key_fsm(&km.lk_sta,rc.mouse.l);
	key_fsm(&km.rk_sta,rc.mouse.r);
	key_fsm(&km.wk_sta,rc.kb.bit.W);
	key_fsm(&km.sk_sta,rc.kb.bit.S);
	key_fsm(&km.ak_sta,rc.kb.bit.A);
	key_fsm(&km.dk_sta,rc.kb.bit.D);	
    chassis_direction_ctrl(CAP_CTRL,SL_SPD,UP,DOWN,LEFT,RIGHT);	   //长摁有效
	chassis_middle_ctrl(KEY_Z);                                                          //Z 点动有效  切换底盘跟随目标
	chassis_rotate_ctrl(KEY_V);
	if (rc.kb.bit.E)
	    rotate_flag = 1;
	if (rc.kb.bit.Q)
	    rotate_flag = 0;
	ramp_mode_ctrl(KEY_R);
	if (rc.kb.bit.Z||rc.kb.bit.V)	
		km.ramp_flag = 0;
	src = rc;	    //记录上一次RC数据
}

void km_global_init(void)
{
  	memset(&km,0,sizeof(kb_ctrl_t));	
}






