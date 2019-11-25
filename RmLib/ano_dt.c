
/** @file ano_dt.c
 *  @version 1.1
 *  @date  Mar 2019
 *
 *  @brief deal with data to draw a waveform
 *
 */
#include "ano_dt.h"
#include "bsp_uart.h"

//#include "hfile.h"
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
extern float out1,out2;
extern float gx,gx2;
extern float AbosoluSpeed;
dt_flag_t f;
uint8_t data_to_send[50];	//·¢ËÍÊý¾Ý»»»º´æ
extern float out1;
//extern pcDataParam pcParam,pcParamLast;
void ANO_DT_Data_Exchange(void)
{
	static uint8_t cnt = 0;
	static uint8_t senser_cnt 	= 10;
	static uint8_t status_cnt 	= 15;
	static uint8_t rcdata_cnt 	= 20;
	static uint8_t motopwm_cnt	= 20;
	static uint8_t power_cnt		=	50;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
//		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
//		ANO_DT_Send_Status(Roll,Pitch,Yaw,baroAlt,0,fly_ready);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
//		ANO_DT_Send_Senser(gimbal.target[1],gimbal.bpit_total_angle,gimbal.buffer[1],
//											 gimbal.bpit_speed_rpm,gimbal.current [2],
//		                                     transport.speed_rpm[0],transport.current[0],
//		                                     transport.speed_rpm[1],transport.current[1],
//		                                     transport.target[0]);
//		ANO_DT_Send_Senser(gimbal.target[2], moto_spit.ecd ,gimbal.buffer[2],
//											 gimbal.spit_speed_rpm,gimbal.current [1],
//		                                     transport.speed_rpm[0],transport.current[0],
//		                                     transport.speed_rpm[1],transport.current[1],
//		                                     transport.target[0]);
//     ANO_DT_Send_Senser(yspeed,YaSpdFiter,AbosoluSpeed/1000,gimbal_yaw.speed_rpm /1000,pcParamLast.pcCompensationX.f*5,0,0,0,0,0);
		
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
//		ANO_DT_Send_RCData(Rc_Pwm_In[0],Rc_Pwm_In[1],Rc_Pwm_In[2],Rc_Pwm_In[3],Rc_Pwm_In[4],Rc_Pwm_In[5],Rc_Pwm_In[6],Rc_Pwm_In[7],0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
//		ANO_DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
//		ANO_DT_Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
//		ANO_DT_Send_PID(1,ctrl_1.PID[PIDROLL].kp,ctrl_1.PID[PIDROLL].ki,ctrl_1.PID[PIDROLL].kd,
//											ctrl_1.PID[PIDPITCH].kp,ctrl_1.PID[PIDPITCH].ki,ctrl_1.PID[PIDPITCH].kd,
//											ctrl_1.PID[PIDYAW].kp,ctrl_1.PID[PIDYAW].ki,ctrl_1.PID[PIDYAW].kd);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
//		ANO_DT_Send_PID(2,ctrl_1.PID[PID4].kp,ctrl_1.PID[PID4].ki,ctrl_1.PID[PID4].kd,
//											ctrl_1.PID[PID5].kp,ctrl_1.PID[PID5].ki,ctrl_1.PID[PID5].kd,
//											ctrl_1.PID[PID6].kp,ctrl_1.PID[PID6].ki,ctrl_1.PID[PID6].kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
//		ANO_DT_Send_PID(3,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
//											ctrl_2.PID[PIDPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
//											ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//	Usb_Hid_Send();					
/////////////////////////////////////////////////////////////////////////////////////
}
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
      HAL_UART_Transmit(&BT_usart,dataToSend,length,20);
}

//void ANO_DT_Data_Receive_Prepare(uint8_t data)
//{
//	static uint8_t RxBuffer[50];
//	static uint8_t _data_len = 0,_data_cnt = 0;
//	static uint8_t state = 0;
//	
//	if(state==0&&data==0xAA)
//	{
//		state=1;
//		RxBuffer[0]=data;
//	}
//	else if(state==1&&data==0xAF)
//	{
//		state=2;
//		RxBuffer[1]=data;
//	}
//	else if(state==2&&data<0XF1)
//	{
//		state=3;
//		RxBuffer[2]=data;
//	}
//	else if(state==3&&data<50)
//	{
//		state = 4;
//		RxBuffer[3]=data;
//		_data_len = data;
//		_data_cnt = 0;
//	}
//	else if(state==4&&_data_len>0)
//	{
//		_data_len--;
//		RxBuffer[4+_data_cnt++]=data;
//		if(_data_len==0)
//			state = 5;
//	}
//	else if(state==5)
//	{
//		state = 0;
//		RxBuffer[4+_data_cnt]=data;
//		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
//	}
//	else
//		state = 0;
//}

//void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
//{
//	uint8_t sum = 0;
//	for(uint8_t i=0;i<(num-1);i++)
//		sum += *(data_buf+i);
//	if(!(sum==*(data_buf+num-1)))		return;		//??sum
//	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
//	
//	if(*(data_buf+2)==0X01)
//	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
//		if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
//	}
//	
//	if(*(data_buf+2)==0X02)
//	{
//		if(*(data_buf+4)==0X01)
//		{
//			f.send_pid1 = 1;
//			f.send_pid2 = 1;
//			f.send_pid3 = 1;
//			f.send_pid4 = 1;
//			f.send_pid5 = 1;
//			f.send_pid6 = 1;
//		}
//		if(*(data_buf+4)==0X02)
//		{
//			
//		}
//		if(*(data_buf+4)==0XA0)		//??????
//		{
//			f.send_version = 1;
//		}
//		if(*(data_buf+4)==0XA1)		//??????
//		{
//			Para_ResetToFactorySetup();
//		}
//	}

//	if(*(data_buf+2)==0X10)								//PID1
//    {
////        ctrl_1.PID[PIDROLL].kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
////        ctrl_1.PID[PIDROLL].ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
////        ctrl_1.PID[PIDROLL].kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
////        ctrl_1.PID[PIDPITCH].kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
////        ctrl_1.PID[PIDPITCH].ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
////        ctrl_1.PID[PIDPITCH].kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
////        ctrl_1.PID[PIDYAW].kp 	= 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
////        ctrl_1.PID[PIDYAW].ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
////        ctrl_1.PID[PIDYAW].kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
//    }
//    if(*(data_buf+2)==0X11)								//PID2
//    {
////        ctrl_1.PID[PID4].kp 	= 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
////        ctrl_1.PID[PID4].ki 	= 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
////        ctrl_1.PID[PID4].kd 	= 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
////        ctrl_1.PID[PID5].kp 	= 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
////        ctrl_1.PID[PID5].ki 	= 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
////        ctrl_1.PID[PID5].kd 	= 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
////        ctrl_1.PID[PID6].kp	  = 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
////        ctrl_1.PID[PID6].ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
////        ctrl_1.PID[PID6].kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
//    }
//    if(*(data_buf+2)==0X12)								//PID3
//    {	
////        ctrl_2.PID[PIDROLL].kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
////        ctrl_2.PID[PIDROLL].ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
////        ctrl_2.PID[PIDROLL].kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
////        ctrl_2.PID[PIDPITCH].kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
////        ctrl_2.PID[PIDPITCH].ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
////        ctrl_2.PID[PIDPITCH].kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
////        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
////        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
////        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
//    }
//	if(*(data_buf+2)==0X13)								//PID4
//	{
//		ANO_DT_Send_Check(*(data_buf+2),sum);
//	}
//	if(*(data_buf+2)==0X14)								//PID5
//	{
//		ANO_DT_Send_Check(*(data_buf+2),sum);
//	}
//	if(*(data_buf+2)==0X15)								//PID6
//	{
//		ANO_DT_Send_Check(*(data_buf+2),sum);
//	}
//}
void ANO_DT_Send_Senser(int16_t target,int16_t angle,float crtout,float speed,int16_t current,int16_t speed1,int16_t current1,int16_t speed2,int16_t current2,int32_t bar)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = target;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = angle;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(int)(crtout*1000) ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(int)(speed*1000) ;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = current;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = speed1;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = current1;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = speed2;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = current2;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
/**
 * @brief  Waveform Drawing
 * @param crtout, angle, speed
 * @return None
 * @attention None
 */
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	ANO_DT_Send_Data(data_to_send, _cnt);
}



