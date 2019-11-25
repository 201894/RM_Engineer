
/** @file Mpu6500.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief read mpu6500 data
 *
 *
 */

#ifndef _MPU6500_H_
#define _MPU6500_H_

#include "stm32f4xx_hal.h"

#include "pid.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
	
}IMUDataTypedef;

extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
extern IMUDataTypedef imu_data_forcal;

uint8_t mpu_device_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
uint8_t IST8310_Init(void);
void IMU_Get_Data(void);

#endif

