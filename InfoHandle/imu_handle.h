/** @file bsp_imu.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief Configuration MPU6500 and Read the Accelerator
 *         and Gyrometer data using SPI interface
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
#ifndef  _IMU_HANDLE_H_
#define _IMU_HANDLE_H_

#include "stm32f4xx_hal.h"
#include <math.h>
#include "pid.h"
#include "Mpu6500.h"


#define IMU_TARGET_TEMP 50

void init_quaternion(void);
void imu_AHRS_update(void);
void InfantryYawUpdate(void);
void PID_Temp_Init(void);
void Temp_keep(void);
void imu_cal_update(void);

extern int imu_init_ok;
extern float q0,q1,q2,q3;
extern float imu_pitch,imu_roll;
extern float  IMU_angle;
extern float imulast,imunow,imu_first;
extern int cnt;
#endif