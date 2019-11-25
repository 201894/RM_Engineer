/** @file imu_handle.h
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief Configuration MPU6500 and Read the Accelerator
 *         and Gyrometer data using SPI interface
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
#include "imu_handle.h"
#include <string.h>
#include "tim.h"
#include "mpu6500_reg.h"
#include "pid.h"
PID_Typedef imu_temp;

float q0 = 1.0,q1 = 0.0,q2 = 0.0,q3 = 0.0;
static volatile uint32_t last_update, now_update;
static volatile float exInt, eyInt, ezInt;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //
float imulast,imunow,imu_first;
float imu_pitch,imu_roll;
int cnt;
int imu_init_ok = 0;

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f375a86 - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

#define BOARD_DOWN 1   //

void init_quaternion(void)
{
  int16_t hx, hy;
  float temp;
	uint8_t buff[4];
	MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,buff,4);
  imu_data_forcal.mx = buff[0]<<8 | buff[1];
	imu_data_forcal.my = buff[2]<<8 | buff[3];
  hx = imu_data_forcal.mx;
  hy = imu_data_forcal.my;
	//printf("hx:%d hy:%d\r\n",hx,hy);
  if (hy != 0)
    temp = hx/hy;
  else
    return ;

  #ifdef BOARD_DOWN
  if(hx<0 && hy <0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.005;
      q1 = -0.199;
      q2 = 0.979;
      q3 = -0.0089;
    }
    else
    {
      q0 = -0.008;
      q1 = -0.555;
      q2 = 0.83;
      q3 = -0.002;
    }
    
  }
  else if (hx<0 && hy > 0) //OK
  {
    if(fabs(temp) >= 1)   
    {
      q0 = 0.005;
      q1 = -0.199;
      q2 = -0.978;
      q3 = 0.012;
    }
    else
    {
      q0 = 0.005;
      q1 = -0.553;
      q2 = -0.83;
      q3 = -0.0023;
    }
    
  }
  else if (hx > 0 && hy > 0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.0012;
      q1 = -0.978;
      q2 = -0.199;
      q3 = -0.005;
    }
    else
    {
      q0 = 0.0023;
      q1 = -0.83;
      q2 = -0.553;
      q3 = 0.0023;
    }
    
  }
  else if (hx > 0 && hy < 0)     //OK
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.0025;
      q1 = 0.978;
      q2 = -0.199;
      q3 = 0.008;
    }
    else
    {
      q0 = 0.0025;
      q1 = 0.83;
      q2 = -0.56;
      q3 = 0.0045;
    }
  }
  #else
    if(hx<0 && hy <0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = 0.195;
      q1 = -0.015;
      q2 = 0.0043;
      q3 = 0.979;
    }
    else
    {
      q0 = 0.555;
      q1 = -0.015;
      q2 = 0.006;
      q3 = 0.829;
    }
    
  }
  else if (hx<0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.193;
      q1 = -0.009;
      q2 = -0.006;
      q3 = 0.979;
    }
    else
    {
      q0 = -0.552;
      q1 = -0.0048;
      q2 = -0.0115;
      q3 = 0.8313;
    }
    
  }
  else if (hx>0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.9785;
      q1 = 0.008;
      q2 = -0.02;
      q3 = 0.195;
    }
    else
    {
      q0 = -0.9828;
      q1 = 0.002;
      q2 = -0.0167;
      q3 = 0.5557;
    }
    
  }
  else if (hx > 0 && hy < 0)
  {
    if(fabs(temp) >= 1)
    {
      q0 = -0.979;
      q1 = 0.0116;
      q2 = -0.0167;
      q3 = -0.195;
    }
    else
    {
      q0 = -0.83;
      q1 = 0.014;
      q2 = -0.012;
      q3 = -0.556;
    }
  }
  #endif
   
}

float halfT;
float Kp  = 2.0, Ki = 0.01;


void imu_AHRS_update(void) 
{
  now_update = HAL_GetTick(); //ms
  halfT =  ((float)(now_update - last_update) / 2000.0f);
  last_update = now_update;
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;//, halfT;
  float tempq0,tempq1,tempq2,tempq3;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  gx = imu_data_forcal.gx / 16.384f / 57.2957795f; // dps->rad/s
  gy = imu_data_forcal.gy / 16.384f / 57.2957795f;
  gz = imu_data_forcal.gz / 16.384f / 57.2957795f;
  ax = imu_data_forcal.ax;
  ay = imu_data_forcal.ay;
  az = imu_data_forcal.az;
  mx = 0;//imu_data_forcal.mx;
  my = 0;//imu_data_forcal.my;
  mz = 0;//imu_data_forcal.mz;


  //Fast inverse square-root
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  /*norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
  hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
  hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz; */

  // estimated direction of gravity and flux (v and w)
  vx = 2.0f*(q1q3 - q0q2);
  vy = 2.0f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
  wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
  wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
      exInt = exInt + ex * Ki * halfT;
      eyInt = eyInt + ey * Ki * halfT;
      ezInt = ezInt + ez * Ki * halfT;
      // PI
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
  }
  // 
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

  //normalise quaternion
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;

}

void InfantryYawUpdate(void){
	
	static int inittic;
	imunow = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3;
	if(imunow - imulast > 330) cnt--;
	if(imunow - imulast < -330) cnt++;
	imulast = imunow;
	if(!imu_init_ok ){
		imu_first = imunow + cnt*360.0;
		inittic++;
		if(inittic > 1000) imu_init_ok  = 1; 
	}
//    ImuData = imunow + cnt*360.0 - imu_first;YUN_Param.Imu_Data
	imu_roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3; // roll       -pi----pi
	imu_pitch = asin(-2*q1*q3 + 2*q0*q2)* 57.3;  
}

void PID_Temp_Init(void){
	memset(&imu_temp,0,sizeof(PID_Typedef));
	imu_temp.kp = 10;
	imu_temp.ki = 1;
	imu_temp.errILim = 150;
	imu_temp.MaxOut = 300;
//	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

void Temp_keep(void){
//	imu_temp.errNow = IMU_TARGET_TEMP - imu_data_forcal.temp;
//	PID_AbsoluteMode(&imu_temp);
	pid_ast(&imu_temp,IMU_TARGET_TEMP,imu_data_forcal.temp);
	TIM3->CCR2 = (uint16_t)imu_temp.ctrOut;
}

void imu_cal_update(void){
//	Temp_keep();
	IMU_Get_Data();
	imu_AHRS_update();
	InfantryYawUpdate();
}
