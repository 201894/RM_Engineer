
/** @file judge_handle.h
 *  @version 2.0
 *  @date April 2019
 *
 *  @brief deal with the judge_system and turn the info into shoot_flag
 *
 */
 
#ifndef __JUDGE_SYSTEM_H__
#define __JUDGE_SYSTEM_H__
#include "stm32f4xx_hal.h"
//#include "bsp_can.h"
//#include "info_get_task.h"
void heat_local_update(uint8_t level);
int mm42heat_ctrl(uint16_t curheat,uint8_t level);
void judge_global_init(void);

#endif
