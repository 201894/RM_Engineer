/** @file chassis_task.h
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief chassis control task
 *
 *
 */
 /*  
      id  0x200 :  M1_LU_ID       201
                        M2_RU_ID      202
						M3_LD_ID      203
						M4_RD_ID      204
*/
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "stm32f4xx_hal.h"

#define  pid_link_adjust               1
#define MAXspeed                          8000 //8000
// Motor 3508 pid
#define  chassis_kp                         6    
#define  chassis_ki                          0.4
#define  chassis_kd                          0
#define  chassis_errILim                   6000
#define  chassis_maxOut                  14500
//pid_link(without gyro)
#define  link_kp                             2.5
#define  link_ki                             0.01
#define  link_kd                           0
#define  link_errILim                     3000
#define  link_maxOut                    5000
//pid_link(with gyro)
#define  link_out_kp                         5
#define  link_out_ki                           0
#define  link_out_kd                          0
#define  link_out_errILim                   0
#define  link_out_maxOut                 1500
#define  link_in_kp                            29
#define  link_in_ki                             0.1
#define  link_in_kd                            0
#define  link_in_errILim                    3000
#define  link_in_maxOut                   30000
extern float power_sum;
extern float power_ratio;
extern float  chassis_speed;

void mecanum_algorithm(float vx,float vy, float vw,int16_t speed[]);
void chassis_param_init(void);
void gyro_algorithm(void);
void chassis_link_handle(void);
#endif
