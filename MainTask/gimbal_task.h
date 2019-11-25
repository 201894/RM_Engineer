
/** @file gimbal_task.h
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief gimbal control task
 *
 *
 */
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "info_handle_task.h"
#include "bsp_can.h"
#define pid_yaw_adjust               0
#define pid_stir_adjust                 0
#define pid_pit_adjust                  1
#define pid_Rpit_adjust                1
#define pid_bullet_adjust              0
#define pid_fric_adjust                  0

#define  stir_out_kp                        15
#define  stir_out_ki                           0
#define  stir_out_kd                          0
#define  stir_out_errILim                   1000
#define  stir_out_maxOut                  400
#define  stir_in_kp                           35
#define  stir_in_ki                             0.08
#define  stir_in_kd                            0
#define  stir_in_errILim                     3000
#define  stir_in_maxOut                   10000

#define  bullet_out_kp                        16
#define  bullet_out_ki                           0
#define  bullet_out_kd                          0
#define  bullet_out_errILim                   1000
#define  bullet_out_maxOut                  800
#define  bullet_in_kp                           40
#define  bullet_in_ki                             0.08
#define  bullet_in_kd                            0
#define  bullet_in_errILim                     3000
#define  bullet_in_maxOut                   10000



#endif
