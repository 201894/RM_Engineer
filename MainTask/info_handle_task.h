/** @file info_handle_task.h
 *  @version 4.0
 *  @date JUNE  2019
 *
 *  @brief get infantry sensor and control information
 *
 *
 */

#ifndef __INFO_HANDLE_TASK_H__
#define __INFO_HANDLE_TASK_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"
#define MOVENORMAL                        13.0f
#define RC_YAW_RATIO                      0.00063 // 0.0001 
typedef enum
{
  SAFETY_MODE	=0,
  NORMAL_MODE,	
  AUTO_MODE,	
  UPSTAIR_MODE,		
} engineer_mode_e;

typedef struct
{
  float         vx; // forward/back
  float         vy; // left/right
  float         vw; // 
  int8_t        mode;
  int32_t       wheel_speed[6];
  int16_t       current[6];
  int16_t       target[6];
  float         buffer[6];
  float         outsum;
  float         UpStairVx;	
  float         ExtraVx;	
  float         currentsum;
  float         link_buffer;	
  float         target_angle;	
}chassis_t;




void moto_info_init(void);
void get_moto_info(void);
void rc_target_handle(rc_info_t *rc);
void can_debug(void);
void get_main_ctrl_mode(void);

extern chassis_t      chassis;

#endif
