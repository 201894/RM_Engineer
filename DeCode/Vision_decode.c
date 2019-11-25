
/** @file Vision_decode.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief vision gimbal control
 *
 *
 */
#include "bsp_uart.h"
#include "pid.h"
#include "STMGood.h"
#include "detect_task.h"
#include "kalman_filter.h"
#include "Vision_decode.h"
#include "CRC_Check.h"
#include "string.h"
#include "gimbal_task.h"
kalman1_state kalmanl;
kalman1_state kalman2;
kalman1_state kalman3;
kalman1_state kalman4;
int ballnum=0,preflag=0;
int vision1flag =0;
float *data = NULL;
int data_len = 0;
float out1 = 0;
int16_t Times = 20000;
float a;
//float buff[22]={0};
//int16_t buffT = 0;
float out2=0;
//int ff = 0;
extern uint8_t uart1_buff[50];
int vlostcount;
float Z=0;
VisionData_t TargetData,TargetLast;
pcDataParam pcParam,pcParamLast;
extern float IMG_KP,IMG_KI,IMG_KD,IMG_ERRILIM,IMG_MAXOUT;
PID_Typedef imgx,imgy;
PID_Typedef predictx;

uint16_t cnt1 = 0;
uint16_t cnt2 = 0;
int flaglose = 1;
int flagshoot = 0;
extern int16_t TmsY ,TmsX;
float sbs = 0;
float H;
float YaSpdFiter;
int flagfd = 0;
float Zbuff[21] = {0};
int ZbuffI = 0;
 int pre_stir_flag;


float buffspeed[8]={0};    //T_Yaw+1;
int16_t buffTimes = 0;
float yspeed=0;  //移相后的yaw轴角速度值
int tt = 0;
extern int shoot_number,number_flag,rotate_flag,Bullet_Number;
void pcDataInit(void)
{
	kalman1_init(&kalmanl,0,100);
	kalman1_init(&kalman2,0,100);
	kalman1_init(&kalman4,0,100);
	kalman13_init(&kalman3,0,100);
	pcParam.isTrue = 0;
	
	pcParam.pcTargetX=0.f;
	pcParam.pcTargetY=0.f;
	pcParam.adjustX=0;
	pcParam.adjustY=0;

	pcParam.CompXout=0;
	pcParam.CompYout=0;
			
	pcParam.refer_centerX=reference_centerX;
	pcParam.refer_centerY=reference_centerY;
}
extern int gyro;
//void Vision_IRQ(void){
//	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET){
//		
//		__HAL_UART_CLEAR_IDLEFLAG(&huart6);		
//		HAL_UART_DMAStop(&huart6);
//		HAL_UART_Receive_DMA(&huart6,uart6_buff,VISIONDATALENGTH);
//					
//	}
//}

void VisionInit(void){
	memset(&TargetData,0,sizeof(VisionData_t));
}
int16_t uart6_cnt=0;

void Vision_Decode(void)
{
	if(uart6_buff[0]==0xA5&& Verify_CRC16_Check_Sum(uart6_buff,VISIONDATALENGTH))//
	{
		pcParam.pcCenterX.uc[0] = uart6_buff[1];
		pcParam.pcCenterX.uc[1] = uart6_buff[2];
		pcParam.pcCenterX.uc[2] = uart6_buff[3];
		pcParam.pcCenterX.uc[3] = uart6_buff[4];
				
		pcParam.pcCenterY.uc[0] = uart6_buff[5];
		pcParam.pcCenterY.uc[1] = uart6_buff[6];
		pcParam.pcCenterY.uc[2] = uart6_buff[7];
		pcParam.pcCenterY.uc[3] = uart6_buff[8];
				
		pcParam.pcCenterZ.uc[0] = uart6_buff[9];
		pcParam.pcCenterZ.uc[1] = uart6_buff[10];
		pcParam.pcCenterZ.uc[2] = uart6_buff[11];
		pcParam.pcCenterZ.uc[3] = uart6_buff[12];
				
		pcParam.pcCompensationX.uc[0] = uart6_buff[13];
		pcParam.pcCompensationX.uc[1] = uart6_buff[14];
		pcParam.pcCompensationX.uc[2] = uart6_buff[15];
		pcParam.pcCompensationX.uc[3] = uart6_buff[16];
				
		pcParam.pcCompensationY.uc[0] = uart6_buff[17];
		pcParam.pcCompensationY.uc[1] = uart6_buff[18];
		pcParam.pcCompensationY.uc[2] = uart6_buff[19];
		pcParam.pcCompensationY.uc[3] = uart6_buff[20];		
/************************球的个数和预置标志位********************************/
        ballnum = uart6_buff[21];
		preflag = uart6_buff[22];
		vision1flag = uart6_buff[23];
		if (uart6_cnt<=100)
		  uart6_cnt++;
	    if (uart6_cnt==1)
		{
		  Bullet_Number =ballnum;
		  number_flag = preflag;				
		}
/*************************************************************************************/
		
/*******目标丢失********************/
		if((pcParam.pcCenterX.f)<5&&(pcParam.pcCenterY.f)<5)
		{
			cnt1++;
			if(cnt1>=2)
			{
				cnt1 = 6;
				flaglose = 1;
				flagshoot = 0;
			}
			else
			{
				flagshoot = 1;
				pcParam.pcCenterX = pcParamLast.pcCenterX;
				pcParam.pcCenterY = pcParamLast.pcCenterY;
				pcParam.pcCenterZ = pcParamLast.pcCenterZ;
				pcParam.pcCompensationX = pcParamLast.pcCompensationX;
				pcParam.pcCompensationY = pcParamLast.pcCompensationY;
			}
		}
/*************找到目标**************/
		else
		{
				cnt1 = 0;
				if(pcParamLast.pcCenterZ.f>0.0f&&pcParam.pcCenterZ.f>0.0f)
				{
					flaglose = 0;
				}
				else
				{
					flaglose = 1;
				}
				flagshoot = 1;			
				pcParamLast.pcCenterX = pcParam.pcCenterX;
				pcParamLast.pcCenterY = pcParam.pcCenterY;
				pcParamLast.pcCenterZ = pcParam.pcCenterZ;
				pcParamLast.pcCompensationX = pcParam.pcCompensationX;
				pcParamLast.pcCompensationY = pcParam.pcCompensationY;			
		}
/*****************云台目标值*********************/
		if(flaglose == 0)
		{
			Z = pcParam.pcCenterZ.f;
			Z = kalman1_filter(&kalman3,Z);                    //目标深度信息，进经过了一次卡尔曼滤波
			pcParam.pcTargetX = pcParam.refer_centerX - pcParam.pcCenterX.f;
			pcParam.pcTargetY = pcParam.pcCenterY.f - reference_centerY ;
			if(pcParamLast.pcCompensationX.f>30)pcParamLast.pcCompensationX.f=0;
			data[0] = pcParamLast.pcCompensationX.f;
			YaSpdFiter = kalman1_filter(&kalmanl,data[0])*8.6;//7.7134;//7.65     //20190729 8.6  //6.6    对目标帧差进行一次卡尔曼滤波，得到目标X方向的速度值
			
			if(abs(pcParam.pcTargetX)<30&&abs(pcParam.pcTargetY)<20)
			{
				flagfd = 1;
			}
			else
			{
				flagfd = 0;
			}
		}
		else  //目标丢失时云台目标值为0
		{
			pcParam.pcTargetX = 0;
			pcParam.pcTargetY = 0;
			Z = 0;
			H = 0;
		}
		  	if(buffTimes>=7||tt==1)
	{
		tt=1;
		if(buffTimes>=7)
			buffTimes = 0;
		yspeed = buffspeed[buffTimes]+0;   //1.5
	}
	else
		yspeed = 0;	
	buffspeed[buffTimes]=gimbal_yaw.speed_rpm;
	buffTimes++;
	if(Z<=4&&Z>2)
	{
		AbosoluSpeed = (YaSpdFiter-yspeed)*10.3f;//(8.4f+2*(Z-2)); 
	}
//	else if(Z<=3&&Z>2)
//	{
//		AbosoluSpeed = (YaSpdFiter-yspeed)*(8.4f+1.1*(Z-2));
//	}
	else if(Z<=2)
	{
		AbosoluSpeed = (YaSpdFiter-yspeed)*8.4f;
	}
	else
		AbosoluSpeed = (YaSpdFiter-yspeed)*13.f;  //6.3
//ANO_DT_Data_Exchange();
	}
	  printf("Distance= %.3f\r\n",Z);
	__HAL_UART_CLEAR_PEFLAG(&huart6);
}


void pc_data_ex(uint8_t mode,uint8_t number,int cycle)
{
    static int cnt;
	  cnt++;
	  pre_stir_flag = number_flag;
// 第一次上电，确保要预置，flag = 1；确保 preflag=1；	
	if (shoot_number)
		pre_stir_flag = 0;
	if (uart6_cnt>=40)
	{
		if (cnt>=cycle)
		{
			cnt = 0;
			uint8_t uart6_data[8];
			uart6_data[0] = 0xA5;
			uart6_data[1] = mode;
			uart6_data[2] = number;
			uart6_data[3] = pre_stir_flag;
			uart6_data[4] = rotate_flag;
			Append_CRC8_Check_Sum(uart6_data,6);
			uart6_data[6] = '\r';
			uart6_data[7] = '\n'; 
			HAL_UART_Transmit(&huart6,uart6_data,8,20);	
		}
    }

}



