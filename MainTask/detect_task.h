
/** @file detect_task.h
 *  @version 1.1
 *  @date Feb 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#ifndef __DETECT_TASK_H__
#define __DETECT_TASK_H__

#include "stm32f4xx_hal.h"
#include "RMsystem.h"
/* detect task period time (ms) */
#define DETECT_TASK_PERIOD 50
#define HERO_FRICOFF  ((uint8_t)(1<<1))
#define HERO_FRICON  ((uint8_t)(1<<7))
/* detect task relevant */
typedef enum
{
  BOTTOM_DEVICE      = 0,
  PIT_GYRO_OFFLINE  = 1,
  YAW_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  REMOTE_CTRL_OFFLINE  = 4,
  JUDGE_SYS_OFFLINE     = 5,
  PC_SYS_OFFLINE            = 6,
  GIMBAL_YAW_OFFLINE   = 7,
  GIMBAL_BPIT_OFFLINE   = 8,
  STIR_M1_OFFLINE           = 9,	
  STIR_M2_OFFLINE           = 10,  	
  STIR_M3_OFFLINE           = 11,  
  BULLET_JAM                     = 12,
  CHASSIS_CONFIG_ERR   = 13,
  GIMBAL_CONFIG_ERR    = 14,
  ERROR_LIST_LENGTH    = 15,
} err_id_e;

typedef  struct
{
	float data[2];
    uint8_t mask;
    struct 
    {
      uint8_t bit0:1;
      uint8_t bit1:1;
      uint8_t bit2:1;
      uint8_t bit3:1;
      uint8_t bit4:1;
      uint8_t bit5:1;
    } light_ctrl;	  
} judge_send_t;

typedef enum
{
  DEV_OFFLINE     = 0,
  DEV_RUNNING_ERR = 1,
  SYS_CONFIG_ERR  = 2,
} err_type_e;

typedef struct
{
  uint16_t set_timeout;
  uint16_t delta_time;
  uint32_t last_time;
} offline_dev_t;

typedef struct
{
  /* enable the device error detect */
  uint8_t  enable;
  /* device error exist flag */
  uint8_t  err_exist;
  /* device error priority */
  uint8_t  pri;
  /* device error type */
  uint8_t  type;
  /* the pointer of device offline param */
  offline_dev_t *dev;
} err_dev_t;

typedef struct
{
  /* the pointer of the highest priority error device */
  err_dev_t *err_now;
  err_id_e  err_now_id;
  /* all device be detected list */
  err_dev_t list[ERROR_LIST_LENGTH];
  /* error alarm relevant */
  uint16_t err_count;
  uint16_t beep_tune;
  uint16_t beep_ctrl;
} global_err_t;

extern global_err_t g_err;
void detector_init(void);
void err_detector_hook(int err_id);
void can_debug(void);
void judge_light_detect(void);
void motor_stall_detect(void);
void detector_param_init(void);
void judge_info_send(float data1,float data2,uint8_t mask);
static void module_offline_callback(void);
static void module_offline_detect(void);
#endif
