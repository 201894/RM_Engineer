
/** @file bsp_can.c
 *  @version 4.0
 *  @date  June 19
 *
 *  @brief receive external can device message
 *
 */

#include "bsp_can.h"
#include "cmsis_os.h"
#include "can.h"
//#include "detect_task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define ENCODER_ANGLE_RATIO               (8192.0f/360.0f)
#define REDUCTION_RATIO                   (36/1)
#define pi                                 3.1415926

uint8_t TxData[8];
CAN_TxHeaderTypeDef  CAN_TxHeader;
//  CAN 1
moto_param    moto_chassis[6];
wl2data       data2bytes;
wl4data       data4bytes; 
gyro_param    gyro_yaw;
extern uint32_t can_cnt[10];
/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */  

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{

  uint8_t RxData1[8],RxData2[8];
  CAN_RxHeaderTypeDef Can1Header,Can2Header;
  if(hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Can1Header,RxData1);
    switch (Can1Header.StdId)
		{
	    case CAN_3508_M1_ID:  
	    case CAN_3508_M2_ID:  
	    case CAN_3508_M3_ID:  
	    case CAN_3508_M4_ID:
	    case CAN_3508_M5_ID:  
	    case CAN_3508_M6_ID:  			
	    {
	       static uint8_t i;
         i = Can1Header.StdId - CAN_3508_M1_ID;			
         moto_chassis[i].speed_rpm = (uint16_t)(RxData1[2] << 8 | RxData1[3]);			 
         can_cnt[0]++;			
	    }break;  	 
			case CAN_YAW_GYRO_ID:
	    {
     	   gyro_data_handle(&data2bytes,&data4bytes,&gyro_yaw,RxData1);		  														
      }break;			
	    default:
      {		 
      }break;
		}
  }
  
}
/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */

void encoder_data_handle(moto_param* ptr,uint8_t RxData[8])
{
   ptr->last_ecd = ptr->ecd;
   ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);  
   if (ptr->ecd - ptr->last_ecd > 4096)
   {
     ptr->round_cnt--;
     ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
   }
   else if (ptr->ecd - ptr->last_ecd < -4096)
   {
     ptr->round_cnt++;
     ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
   }
   else
   {
     ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
   }
   ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
   /* total angle, unit is degree */
   ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
   ptr->stir_angle =   (float)ptr->total_angle/REDUCTION_RATIO;
  
   ptr->speed_rpm     = (int16_t)(RxData[2] << 8 | RxData[3]);
   ptr->current = (int16_t)(RxData[2] << 8 | RxData[3]);
}

/**
  * @brief     get gyro data and unpack the data 
* @param     ptr: Pointer to a wl2data structure  ptrr: Pointer to a wl4data structure
  * @attention this function should be called after gyro is read
  */
void gyro_data_handle(wl2data* ptr,wl4data* ptrr,gyro_param* gyro,uint8_t RxData[8])
{
//	    gyro->yaw_speed_last = gyro->yaw_speed;
//	    gyro->pit_speed_last = gyro->pit_speed;
	  ptr->c[0] = RxData[0];
		ptr->c[1] = RxData[1];
	    gyro->pit_speed = -(float)ptr->d / 16.384f;
//  	if (abs(gyro->pit_speed_last-gyro->pit_speed)>2000)
//		    gyro->pit_speed = gyro->pit_speed_last;	   
		ptr->c[0] = RxData[2];
		ptr->c[1] = RxData[3];    
		gyro->yaw_speed  = -(float)ptr->d / 16.384f;
//	    if (abs(gyro->yaw_speed_last-gyro->yaw_speed)>2000)
//			gyro->yaw_speed = gyro->yaw_speed_last;
		ptrr->c[0] = RxData[4];
		ptrr->c[1] = RxData[5];
		ptrr->c[2] = RxData[6];
		ptrr->c[3] = RxData[7];
		gyro->angle_z = ptrr->f ;	
}

void super_data_handle(wl4data* ptr1,uint8_t RxData[8])
{
		ptr1->c[0] = RxData[0];
		ptr1->c[1] = RxData[1];
		ptr1->c[2] = RxData[2];
		ptr1->c[3] = RxData[3];
}

/**
  * @brief  send calculated current to motor
  * @param  3508 motor ESC id
  */
void send_chassis_cur(uint32_t id,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	  CAN_TxHeader.StdId    = id;
	  CAN_TxHeader.IDE      = CAN_ID_STD;
	  CAN_TxHeader.RTR      = CAN_RTR_DATA;
	  CAN_TxHeader.DLC      = 0x08;
	  TxData[0] = iq1 >> 8;
	  TxData[1] = iq1;
	  TxData[2] = iq2 >> 8;
	  TxData[3] = iq2;
	  TxData[4] = iq3 >> 8;
	  TxData[5] = iq3;
	  TxData[6] = iq4 >> 8;
	  TxData[7] = iq4;
    HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeader,TxData,(uint32_t *)CAN_TX_MAILBOX0);
}


void send_chassis_ms(uint32_t id,uint8_t data[8])
{
	  CAN_TxHeader.StdId    = id;
	  CAN_TxHeader.IDE      = CAN_ID_STD;
	  CAN_TxHeader.RTR     = CAN_RTR_DATA;
	  CAN_TxHeader.DLC     = 0x08;
	  TxData[0] = data[0];
	  TxData[1] = data[1];
	  TxData[2] = data[2];
	  TxData[3] = data[3];
	  TxData[4] = data[4];
	  TxData[5] = data[5];
	  TxData[6] = data[6];
	  TxData[7] = data[7];
    HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeader,TxData,(uint32_t *)CAN_TX_MAILBOX0);
}
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterTypeDef  can_filter;
	
  can_filter.FilterActivation     = ENABLE;
  can_filter.FilterBank         = 0;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank      = 14;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
	
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
}

void can_receive_start(void)
{
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}



