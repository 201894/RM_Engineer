/** @file gimbal_task.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief gimbal control task
 *
 *
 */
#include "bsp_can.h"
#include "pid.h" 
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "Vision_decode.h"
#include "STMGood.h"
#include "math.h"
#include "string.h"
#include "info_handle_task.h"
/*YAW轴电机PID参数*/
#define  yaw_out_kp                        25
#define  yaw_out_ki                           0
#define  yaw_out_kd                          0
#define  yaw_out_errILim                   400
#define  yaw_out_maxOut                 1000        //800
#define  yaw_in_kp                           145
#define  yaw_in_ki                            0.2
#define  yaw_in_kd                           0
#define  yaw_in_errILim                    20000
#define  yaw_in_maxOut                  28000
/*左边P轴电机PID参数*/
#define  pit_out_kp                         0
#define  pit_out_ki                           0
#define  pit_out_kd                          0
#define  pit_out_errILim                   0
#define  pit_out_maxOut                 800
#define  pit_in_kp                          0
#define  pit_in_ki                           0
#define  pit_in_kd                            0
#define  pit_in_errILim                     6000
#define  pit_in_maxOut                    25000
/*右边P轴电机PID参数*/
#define  Rpit_out_kp                          0
#define  Rpit_out_ki                           0
#define  Rpit_out_kd                          0
#define  Rpit_out_errILim                   0
#define  Rpit_out_maxOut                 800
#define  Rpit_in_kp                          0
#define  Rpit_in_ki                             0
#define  Rpit_in_kd                            0
#define  Rpit_in_errILim                     6000
#define  Rpit_in_maxOut                    25000
/*摩擦轮电机PID参数*/
#define  fric_kp                                20.0f
#define  fric_ki                                  0.0f
#define  fric_kd                                 0.0f
#define  fric_errILim                          3000
#define  fric_maxOut                         15000

gimbal_t        gimbal_pit;
gimbal_t        gimbal_Rpit;
gimbal_t        gimbal_yaw;
int16_t fric_speed;
uint8_t TxData2[8];
extern uint8_t tem_6020;
extern float yaw_target,pit_target;
extern int flaglose,rotate_flag;
/***********视觉相关参数定义*********************/
uint8_t  data405[8];
//float buffspeed[8]={0};    //T_Yaw+1;
//int16_t buffTimes = 0;
//float yspeed=0;  //移相后的yaw轴角速度值
//int tt = 0;
/*****************************************************/
void gimbal_task(void const *argu)
{
   for(;;)
   {
//	   	if(buffTimes>=7||tt==1)
//	{
//		tt=1;
//		if(buffTimes>=7)
//			buffTimes = 0;
//		yspeed = buffspeed[buffTimes];
//	}
//	else
//		yspeed = 0;	
//	buffspeed[buffTimes]=gimbal_yaw.speed_rpm;
//	buffTimes++;
	 
     if(gimbal.mode)
	 {
		/***************云台电机PID参数调试**********************************************/
	  #if pid_stir_adjust
	    pid_adjust(&pid_stir_out,_kp,_ki,_kd);
	    pid_adjust(&pid_stir_in,_kkp,_kki,_kkd);
	    pid_stir_out.MaxOut = maxout1;
	    pid_stir_in.MaxOut   = maxout2;
      #endif
	  #if pid_bullet_adjust
	    pid_adjust(&pid_bullet_out,_kp,_ki,_kd);
	    pid_adjust(&pid_bullet_in,_kkp,_kki,_kkd);
			pid_bullet_out.MaxOut = maxout1;
	    pid_bullet_in.MaxOut   = maxout2;
      #endif
	  #if pid_fric_adjust
	    pid_adjust(&pid_fric[0],_kp,_ki,_kd);
	    pid_adjust(&pid_fric[1],_kp,_ki,_kd);
	  #endif	
		 
		/***************摩擦轮速度环PID**************************************************/
		pid_ast(&pid_fric[0],fric_speed,moto_fric[0].speed_rpm);
		pid_ast(&pid_fric[1],-fric_speed,moto_fric[1].speed_rpm);//fricion
		/********************************************************************************/
		
		/***************弹丸拨盘双环PID**************************************************/
		pid_ast(&pid_bullet_out,kernel_bullet.target,kernel_bullet.total_angle);//shoot big bullets
		kernel_bullet.buffer = pid_bullet_out.ctrOut;
		pid_ast(&pid_bullet_in,kernel_bullet.buffer,kernel_bullet.speed_rpm);//shoot big bullets 
		kernel_bullet.current = (int)pid_bullet_in.ctrOut;
		/********************************************************************************/
		
		/***************CAN2发送摩擦轮、弹丸拨盘控制量**************************************/
		send_gimbal_cur(0x200,(int)pid_fric[0].ctrOut,(int)pid_fric[1].ctrOut,kernel_bullet.current,0);
		/********************************************************************************/
		
		/***************弹丸转盘双环PID**************************************************/
		pid_ast(&pid_stir_out,kernel_stir.target,kernel_stir.total_angle);//stir big bullets
		kernel_stir.buffer = pid_stir_out.ctrOut;
		pid_ast(&pid_stir_in,kernel_stir.buffer,kernel_stir.speed_rpm);//stir big bullets	
		kernel_stir.current = (int)pid_stir_in.ctrOut;
		/********************************************************************************/

		/***************YAW轴双环PID*****************************************************/
		yaw_identity_handle(flaglose,gimbal.mode);
		gimbal_yaw.buffer = pid_yaw_out.ctrOut+IMG_YAW_OUT.ctrOut;
		pid_ast(&pid_yaw_in,gimbal_yaw.buffer,gimbal_yaw.speed_rpm*2.5);  		
		gimbal_yaw.current = (int)pid_yaw_in.ctrOut;
		/********************************************************************************/
		
		/***************CAN1发送YAW轴控制量、弹丸转盘控制量******************************/
	    send_chassis_cur(0x1ff,gimbal_yaw.current,kernel_stir.current,0,0);		
		/********************************************************************************/
		
		/***************双P轴双环PID*****************************************************/
		pit_identity_handle(flaglose,gimbal.mode);
		gimbal_pit.buffer = (-pid_Rpit_out.ctrOut -IMG_PIT_OUT.ctrOut);
		gimbal_Rpit.buffer =  (pid_Rpit_out.ctrOut + IMG_PIT_OUT.ctrOut);
		pid_ast(&pid_pit_in,gimbal_pit.buffer,-gyro_pit.pit_speed);    // LEFT PIT
		pid_ast(&pid_Rpit_in,gimbal_Rpit.buffer,gyro_pit.pit_speed);	//  RIGHT PIT	
		gimbal_pit.current = (int)pid_pit_in.ctrOut;
		gimbal_Rpit.current = (int)pid_Rpit_in.ctrOut;
		/********************************************************************************/
		
		/***************CAN2发送双P轴控制量**********************************************/
	    if (tem_6020<=50)
		  send_gimbal_cur(0x1ff,gimbal_pit.current,gimbal_Rpit.current,0,0);
		else
		  send_gimbal_cur(0x1ff,0,0,0,0);
		/********************************************************************************/
	}
	 else
	 {
		send_gimbal_cur(0x1ff,0,0,0,0);
		send_gimbal_cur(0x200,0,0,0,0);		 
	 }
	
	data405[0] =0;
	send_405(0x305,data405);
    osDelay(5);
   }

}

float AbosoluSpeed = 0;
int ff = 0;
float xlose=0;
float TargetXLast=0,TargetYLast=0;
float XBase = 0;
float aspeed=0;
int16_t buffT = 0;
int16_t TmsX = 0,TmsY = 0;
extern float out1,YaSpdFiter,yspeed,Z,H;

void pit_identity_handle(int identity_flag,int mode_flag)
{
	if (mode_flag==AUTO_MODE)
	{
	    if(identity_flag!=0)
		{
		  	IMG_PIT_OUT.ctrOut = 0;				
		    pid_adjust(&pid_Rpit_out,26,0,0);				
         if (km.ramp_flag==0)			
            pid_ast(&pid_Rpit_out,(9684-gimbal_pit.target) *0.04394531f,moto_Rpit.ecd*0.04394531f);
         else 
            pid_ast(&pid_Rpit_out,(9684-pit_target) *0.04394531f,moto_Rpit.ecd*0.04394531f);	
			pid_adjust(&pid_pit_in,100,0.2,0);
		    pid_adjust(&pid_Rpit_in,100,0.2,0);		
		}
		else
		{
			pid_pit_out.ctrOut = 0;		
			pid_Rpit_out.ctrOut = 0;
			/**************串口TTL掉线一异常处理方法****************/
			if(TargetYLast==pcParam.pcCenterY.f)
			{
				TmsY++;
			}
			else
			{
				TmsY = 0;
			}
			if(TmsY>=5)
			{
				TmsY = 5;
				identity_flag = 1;
			}
			/***************************************************************/
			if(Z<=0.6)
			{
				H = 50*180*asin(9.8*Z/225);
			}
			else if(Z<=1.0&&Z>0.6)
			{
				H = (50-75*(Z-0.6))*180*asin(9.8*Z/225);
//				H = _ki*180*asin(9.8*Z/225);				
			}
			else if(Z<=1.3&&Z>1.0)
			{
				H = (20 -10*(Z-1.0))*180*asin(9.8*Z/225);
//				H = _kd*180*asin(9.8*Z/225);					
			}
			else if(Z<=1.5&&Z>1.3)
			{
				H = (46.25-22.5*Z)*180*asin(9.8*Z/225);								
//				H = (10.5-8*(Z-1.5))*180*asin(9.8*Z/225);
			}
			else if(Z<=2.0&&Z>1.5)
			{
				H = 12.5*180*asin(9.8*Z/225);								
//				H = (10.5-8*(Z-1.5))*180*asin(9.8*Z/225);
			}
			else if(Z<=2.8&&Z>2.0)
			{
				H = (12-3.2*(Z-2))*180*asin(9.8*Z/225);
//				H = 10.5*180*asin(9.8*Z/225);
			}
			else if(Z<=4.5&&Z>2.8)
			{
				H = (-0.06*Z+9.61)*180*asin(9.8*Z/225);
			}
//			else if (Z<=4&&Z>3.5)
//			{
//				H = (-1.26*Z+13.54)*180*asin(9.8*Z/225);
//			}
			else
			{
				H =  9*180*asin(9.8*Z/225);
			}
	        pid_adjust(&IMG_PIT_OUT,1.5,0,0);			//5.5		
			pid_adjust(&pid_pit_in,50,0.1,0);
		    pid_adjust(&pid_Rpit_in,50,0.1,0);			 
		    pid_ast(&IMG_PIT_OUT,435-60+H*0.62,pcParam.pcCenterY.f);	
			gimbal_pit.target = 9684-moto_Rpit.ecd;
		}
	}
	else
	{
		  	IMG_PIT_OUT.ctrOut = 0;				
		    pid_adjust(&pid_Rpit_out,26,0,0);
         if (km.ramp_flag==0)			
            pid_ast(&pid_Rpit_out,(9684-gimbal_pit.target) *0.04394531f,moto_Rpit.ecd*0.04394531f);
         else 
            pid_ast(&pid_Rpit_out,(9684-pit_target) *0.04394531f,moto_Rpit.ecd*0.04394531f);			 
			pid_adjust(&pid_pit_in,100,0.2,0);
		    pid_adjust(&pid_Rpit_in,100,0.2,0);
	}
}


/***********************************移相********************************/
//float buffspeed[11]={0};    //T_Yaw+1;
//int16_t buffTimes = 0;
//float yspeed=0;  //移相后的yaw轴角速度值
//int tt = 0;
//void dephasing(int T_Yaw)
//{
//	if(buffTimes>=T_Yaw||tt==1)
//	{
//		tt=1;
//		if(buffTimes>=T_Yaw)
//			buffTimes = 0;
//		yspeed = buffspeed[buffTimes];
//	}
//	else
//		yspeed = 0;	
//	buffspeed[buffTimes]=gimbal_yaw.speed_rpm;
//	buffTimes++;
//}
/*************************************************************************/
void yaw_identity_handle(int identity_flag,int mode_flag)
{
	if (mode_flag==AUTO_MODE)
	{
		if(identity_flag!=0)
		{
			AbosoluSpeed=0;
			IMG_YAW_OUT.ctrOut = 0;
			//失去目标时由这套PID控制YAW轴；
			pid_adjust(&pid_yaw_out,30,0,0);
			pid_adjust(&pid_yaw_in,159,0.6,0);
         if (km.ramp_flag==0)			
			pid_ast(&pid_yaw_out,gimbal_yaw.target,gimbal_yaw.total_angle);
		 else
			pid_ast(&pid_yaw_out,yaw_target,gimbal_yaw.total_angle);			 
		}
		else  //捕获目标时；
		{
			pid_yaw_out.ctrOut = 0;			
			      //相对速度YaSpdFiter加上云台速度yspeed即得到目标的绝对速度；
			/**************串口TTL掉线一异常处理方法****************/
			if(xlose==pcParam.pcCenterX.f)
			{
				TmsX++;
			}
			else
			{
				TmsX = 0;
				flaglose = 0;
				xlose = pcParam.pcCenterX.f;
			}
			if(TmsX>=5)
			{
				TmsX = 5;
				AbosoluSpeed = 0;
				identity_flag = 1;
				flaglose = 1;
			}
			/***************************************************************/		
			/*******************矫正因为相机轴线不在枪管轴线上所带来的YAW轴基准值偏差*************************/
			if(Z<0.85&&Z>=0.6)
			{
				XBase = 465;
			}
			else if(Z<0.6)
			{
				XBase = 450;
			}
			else if(Z>=0.85&&Z<1.18)
			{
				XBase = 465+(Z-0.85)*67;
			}
			else if(Z>=1.18&&Z<1.63)
			{
				XBase = 487+(Z-1.18)*33.7;
			}
			else if(Z>=1.63&&Z<1.99)
			{
				XBase = 502+(Z-1.63)*21.92;
			}
			else if(Z>=1.99&&Z<2.43)
			{
				XBase = 510+(Z-1.99)*18.31;
			}
			else if(Z>=2.43)
			{
				XBase = 518+(Z-2.43)*7.03;
			}
			/**************************************************************************************************************************/
			if(rotate_flag==1)
			{
		      pid_adjust(&IMG_YAW_OUT,2.0,0,0);		//2;	
		      pid_adjust(&pid_yaw_in,55,0.3,0);	//145,0.2;				
			  pid_ast(&IMG_YAW_OUT,0.96*XBase,(pcParam.pcCenterX.f)); // 0.97 
			}
			else
			{
		      pid_adjust(&IMG_YAW_OUT,1.8,0,0);		//2;	
		      pid_adjust(&pid_yaw_in,47,0.21,0);	//145,0.2;
			  if (!km.rotate_flag)
			  {
			   if(AbosoluSpeed>0)
			    pid_ast(&IMG_YAW_OUT,0.96*XBase,(float)(pcParam.pcCenterX.f+(float)AbosoluSpeed));
			   else if(AbosoluSpeed<=0)
			   {
				  AbosoluSpeed =-1*AbosoluSpeed;
				  pid_ast(&IMG_YAW_OUT,0.96*XBase,(float)(pcParam.pcCenterX.f-(float)AbosoluSpeed));
			   }
		      }
			  else
			  {
			   if(AbosoluSpeed>0)
			    pid_ast(&IMG_YAW_OUT,1.04*XBase,(float)(pcParam.pcCenterX.f+(float)AbosoluSpeed));  //0.83
			   else if(AbosoluSpeed<=0)
			   {
				  AbosoluSpeed =-1*AbosoluSpeed;
				  pid_ast(&IMG_YAW_OUT,1.04*XBase,(float)(pcParam.pcCenterX.f-(float)AbosoluSpeed)); //1.04
			   }
		      }				  
		  }
//			pid_ast(&IMG_YAW_OUT,(0.905*XBase+0*AbosoluSpeed),pcParam.pcCenterX.f); // 0.97 			
			gimbal_yaw.target =gimbal_yaw.total_angle;
		}
	}
	else
	{
		AbosoluSpeed=0;
		IMG_YAW_OUT.ctrOut = 0;
		//失去目标时由这套PID控制YAW轴；
		pid_adjust(&pid_yaw_out,30,0,0);
		pid_adjust(&pid_yaw_in,159,0.6,0);		
         if (km.ramp_flag==0)			
			pid_ast(&pid_yaw_out,gimbal_yaw.target,gimbal_yaw.total_angle);
		 else
			pid_ast(&pid_yaw_out,yaw_target,gimbal_yaw.total_angle);
	}
}

void gimbal_param_init(void)
{
	PID_struct_init(&pid_stir_out,stir_out_errILim,stir_out_maxOut,\
	  stir_out_kp,stir_out_ki,stir_out_kd);  //             2006                  STIR	
	PID_struct_init(&pid_stir_in,stir_in_errILim,stir_in_maxOut,\
	  stir_in_kp,stir_in_ki,stir_in_kd);   //                   2006                  STIR	

	PID_struct_init(&pid_bullet_out,bullet_out_errILim,bullet_out_maxOut,\
	  bullet_out_kp,bullet_out_ki,bullet_out_kd);  //    2006                  BULLET			
	PID_struct_init(&pid_bullet_in,bullet_in_errILim,bullet_in_maxOut,\
	  bullet_in_kp,bullet_in_ki,bullet_in_kd);   //          2006                  BULLET		
	
	PID_struct_init(&pid_fric[0],fric_errILim,fric_maxOut,fric_kp,fric_ki,fric_kd);  //3508       FRIC  LEFT
	    PID_struct_init(&pid_fric[1],fric_errILim,fric_maxOut,fric_kp,fric_ki,fric_kd);  //3508    FRIC  RIGHT
		
	PID_struct_init(&pid_yaw_out,yaw_out_errILim,yaw_out_maxOut,\
	  yaw_out_kp,yaw_out_ki,yaw_out_kd);   //       6020                  YAW
	PID_struct_init(&pid_yaw_in,pit_in_errILim,pit_in_maxOut,\
	  yaw_in_kp,yaw_in_ki,yaw_in_kd);  //              6020                   YAW 

	PID_struct_init(&pid_pit_out,pit_out_errILim,pit_out_maxOut,\
  	  pit_out_kp,pit_out_ki,pit_out_kd);  //                6020                   PIT	
	PID_struct_init(&pid_pit_in,pit_in_errILim,pit_in_maxOut,\
	  pit_in_kp,pit_in_ki,pit_in_kd);   //                      6020                   PIT	
	
	PID_struct_init(&pid_Rpit_out,Rpit_out_errILim,Rpit_out_maxOut,\
  	  Rpit_out_kp,Rpit_out_ki,Rpit_out_kd);  //                6020                   RPIT	
	PID_struct_init(&pid_Rpit_in,Rpit_in_errILim,Rpit_in_maxOut,\
	  Rpit_in_kp,Rpit_in_ki,Rpit_in_kd);   //                      6020                   RPIT		
     IMG_PIT_OUT.MaxOut = 500;
     IMG_RPIT_OUT.MaxOut = 500;
     IMG_YAW_OUT.errILim = 200; 	 
	 IMG_YAW_OUT.MaxOut = 320;
}
