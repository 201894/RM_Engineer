

#include "Mpu6500.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "spi.h"
#include "freertos.h"
#include "kalman_filter.h"
#include "gpio.h"
#include <string.h>

kalman1_state kalman_imu;
uint8_t MPU_id = 0;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_forcal = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offset = {0,0,0,0,0,0,0,0,0,0};


//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();

  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}


void IMU_Get_Data()
{
    uint8_t mpu_buff[14];
    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H , mpu_buff, 14);//14
	imu_data_forcal.ax = mpu_buff[0]<<8 |mpu_buff[1];
	imu_data_forcal.ay = mpu_buff[2]<<8 |mpu_buff[3];
	imu_data_forcal.az = mpu_buff[4]<<8 |mpu_buff[5];
	imu_data_forcal.temp = 21 + (mpu_buff[6]<<8 |mpu_buff[7])/333.87f;	
	
	imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9];
	imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11];
	imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13];
	
	imu_data_forcal.gx = imu_data.gx - imu_data_offset.gx;
	imu_data_forcal.gy = imu_data.gy - imu_data_offset.gy;
	imu_data_forcal.gz = imu_data.gz - imu_data_offset.gz;
	
//	if (abs(imu_data_forcal.gx)<=10)  imu_data_forcal.gx = 0;
//	if (abs(imu_data_forcal.gy)<=10)  imu_data_forcal.gy = 0;
//	if (abs(imu_data_forcal.gz)<=10)  imu_data_forcal.gz = 0;	
	
//	 YUN_Param.Pitch_Speed   = - imu_data.gy * 0.06103515625F;  // y
   //    imu_data_forcal.gy = kalman1_filter(&kalman_imu,imu_data_forcal.gy);
//	 YUN_Param.YAW_Speed =  -imu_data.gz * 0.06103515625F;
}



static void mpu_offset_init(void){
	uint8_t mpu_buff[6];
	int offset_gx = 0,offset_gy = 0,offset_gz = 0;
	for(int i=0;i<300;i++){
		MPU6500_Read_Regs(MPU6500_GYRO_XOUT_H, mpu_buff, 6);//14
		imu_data_offset.gx = mpu_buff[0]<<8 |mpu_buff[1];
		imu_data_offset.gy = mpu_buff[2]<<8 |mpu_buff[3];
		imu_data_offset.gz = mpu_buff[4]<<8 |mpu_buff[5];	
		offset_gx += imu_data_offset.gx;
		offset_gy += imu_data_offset.gy;
		offset_gz += imu_data_offset.gz;
		HAL_Delay(3);
	}
	imu_data_offset.gx = offset_gx / 300;
	imu_data_offset.gy = offset_gy / 300;
	imu_data_offset.gz = offset_gz / 300;

}
uint8_t mpu_device_Init(void)
{
  memset(&imu_data,0,sizeof(IMUDataTypedef));
  memset(&imu_data_forcal,0,sizeof(IMUDataTypedef));
  memset(&imu_data_offset,0,sizeof(IMUDataTypedef));
  PID_Temp_Init();
  ENABLE_IST;
  MPU6500_Write_Reg(MPU6500_PWR_MGMT_1, 0x80);
  HAL_Delay(200);
  MPU6500_Write_Reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  HAL_Delay(200);
  if (MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I)){
	  HAL_Delay(400);
	  if (MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I)){
		printf("imu offline!\r\n");
		return 1;
	  }
  }
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[7][2] = {
    { MPU6500_PWR_MGMT_1,     0x03 }, // Auto selects Clock Source
    { MPU6500_PWR_MGMT_2,     0x00 }, // all enable
    { MPU6500_CONFIG,         0x04 }, // gyro bandwidth 184Hz 01
    { MPU6500_GYRO_CONFIG,    0x18 }, // +-2000dps
    { MPU6500_ACCEL_CONFIG,   0x10 }, // +-8G
    { MPU6500_ACCEL_CONFIG_2, 0x04 }, // acc bandwidth 20Hz
    { MPU6500_USER_CTRL,      0x20 }, // Enable the I2C Master I/F module
                                      // pins ES_DA and ES_SCL are isolated from 
                                      // pins SDA/SDI and SCL/SCLK.
  };
  for(index = 0; index < 7; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }
  IST8310_Init();
  mpu_offset_init();
  init_quaternion();
  	kalman1_init(&kalman_imu,0,100);
  return 0;
}




