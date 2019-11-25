
/** @file bsp_can.h
 *  @version 4.0
 *  @date  June 2019
 *
 *  @brief receive external can device message
 *
 */

#ifndef  __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  CAN_3508_M1_ID                    = 0x201,
  CAN_3508_M2_ID                    = 0x202,
  CAN_3508_M3_ID                    = 0x203,
  CAN_3508_M4_ID                    = 0x204,
  CAN_3508_M5_ID                    = 0x205,
  CAN_3508_M6_ID                    = 0x206,	
  CAN_STIR_ID	                          = 0x206,
  CAN_YAW_ID	                      = 0x205,	
  EXT_CAP_INFO                      = 0x308,	
  EXT_GAME_STATE                 = 0x101,
  EXT_ROBOT_STATE               = 0x102,	
  EXT_POWER_DATA                = 0x103,
  EXT_HEAT_DATA                    = 0x104,
  EXT_XY_POSITION                 = 0x105,
  EXT_ZY_POSITION                 = 0X106,
  EXT_BUFF_MUSK                   = 0x107,
  EXT_ROBOT_HURT                = 0x111,
  EXT_SHOOT_DATA                 = 0x112,	
} can_msg_id1;
typedef enum
{
	CAN_PIT_GYRO_ID                = 0x101,  
  CAN_3508_FL_ID                    = 0x201,
  CAN_3508_FR_ID                    = 0x202,
  CAN_BULLET_ID                     = 0x203,	
	CAN_PIT_ID                            = 0x205,
  CAN_RPIT_ID                          = 0x206,  
  CAN_YAW_GYRO_ID              = 0x401, 	
} can_msg_id2;
typedef union
{
	uint8_t c[2];
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union
{	
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;

typedef struct
{
	float pit_speed;
	float angle_z;
	float yaw_speed;
	float yaw_speed_last;	
	float pit_speed_last;		
}gyro_param;


/* can receive motor parameter structure */
#define FILTER_BUF 5
typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  uint16_t temp;	
  int16_t  speed_rpm;
  int16_t  current;
  int16_t  torque;	
  int32_t  round_cnt;
  int32_t  total_ecd;
  float      total_angle;
  float      stir_angle;
  float      angle_offset;
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
	
} moto_param;

typedef __packed struct 
{  
	uint8_t game_type : 4; /*1 机甲大师赛   
												  *2  单项赛   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  未开始比赛 
														 *1  准备阶段  
														 *2  自检阶段
														 *3  5s倒计时 
														 *4  对战中
														 *5  比赛结算中  */
	uint16_t stage_remain_time; /* 当前阶段剩余时间 s*/
} ext_game_state_t; 

typedef __packed struct{ 
	uint8_t winner; /* 0 平局
									 * 1 红方胜利
									 * 2 蓝方胜利 */
}ext_game_result_t;

typedef __packed struct{
	uint16_t robot_legion; /* 红方 位0开始 英雄、工程、步兵1、步兵2、步兵3、空中、哨兵、保留 
												  * 蓝方 位8开始 英雄、工程、步兵1、步兵2、步兵3、空中、哨兵、保留 */
}ext_game_robot_survivors_t;

typedef __packed struct{
	uint32_t event_type; /* 无用 如需要，查手册 */
}ext_event_data_t;

typedef __packed struct { 
	uint8_t supply_projectile_id;/* 补给站ID    
															  * 1：1号补给口 
															  * 2：2号补给口 */
	uint8_t supply_robot_id;/* 补弹机器人ID   
													 * 0 ： 无机器人补弹 
													 * 1 ： 红英雄
													 * 2 ： 红工程
													 * 3/4/5 ： 红步兵
													 * 11 ： 蓝英雄
													 * 12 ： 蓝工程
													 * 13/14/15 ： 蓝步兵		*/
	uint8_t supply_projectile_step;/* 出弹口开闭状态 
																  * 0：关闭
																  * 1：子弹准备中
																  * 2：子弹下落 */
	uint8_t supply_projectile_num;  /* 补弹数量  50/100/150/200 */
} ext_supply_projectile_action_t;

typedef __packed struct 
{  
	uint8_t robot_id;/* 机器人ID 
									  * 1： 红英雄
									  * 2： 红工程
									  * 3/4/5：红步兵
									  * 6： 红空中
									  * 7： 红哨兵
										  * 11：蓝英雄
										  * 12：蓝工程
										  * 13/14/15：蓝步兵		
										  * 16  蓝空中
										  * 17  蓝哨兵*/
	uint8_t robot_level; /* 机器人等级 1/2/3*/
	uint16_t remain_HP; /* 机器人剩余血量 */
	uint16_t max_HP; /* 机器人上限血量 */
	uint16_t shooter_heat0_cooling_rate; /* 机器人17mm枪口每秒冷却值 */
	uint16_t shooter_heat0_cooling_limit; /* 机器人17mm枪口热量上限 */
	uint16_t shooter_heat1_cooling_rate;	/* 机器人42mm枪口每秒冷却值 */
	uint16_t shooter_heat1_cooling_limit; /* 机器人42mm枪口热量上限 */
	uint8_t mains_power_gimbal_output : 1; /* 云台电源输出情况 */
	uint8_t mains_power_chassis_output : 1; /* 底盘电源输出情况 */
	uint8_t mains_power_shooter_output : 1; /* 发射摩擦轮电源输出情况 */
} ext_game_robot_state_t; 

//0x0103	请求补给站补弹 对抗赛未开放
typedef __packed struct {
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;

typedef __packed struct 
{   
	uint16_t chassis_volt; /* 底盘输出电压 mV */
	uint16_t chassis_current; /* 底盘输出电流 mA*/
	float chassis_power; /* 底盘输出功率 W */
	uint16_t chassis_power_buffer; /* 底盘功率缓冲 J */
	uint16_t shooter_heat0; /* 17mm枪口热量 */
	uint16_t shooter_heat1; /* 42mm枪口热量 */

/*	                        /s                          /s
level     17mm      cd            42mm    cd
  1         240         40            200       20  
  2         360         60            300       40
  3	     480         80            400       60
*/	
	
} ext_power_heat_data_t; 

typedef __packed struct {
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204 机器人增益
typedef __packed struct { 
	uint8_t power_rune_buff; /* bit0  补血状态 
													  * bit1  热量冷却加倍
													  * bit2  防御加成
													  * bit3  攻击加成 
													  * 其余保留 */
}ext_buff_musk_t;

//0x0206 伤害状态
typedef __packed struct {
	uint8_t armor_id : 4; /* 装甲伤害时为装甲ID 其余为0 */
	uint8_t hurt_type : 4; /* 0 装甲伤害 
												  * 1 模块离线扣血 
												  * 2 枪口超热量扣血 
												  * 3 底盘超功率扣血 */
} ext_robot_hurt_t;

//0x0207 实时射击信息
typedef __packed struct {
	uint8_t bullet_type; /* 子弹类型 17mm为1 42mm为2*/
	uint8_t bullet_freq; /* 射频 */
	float bullet_speed; /* 射速 */
} ext_shoot_data_t;

//0x0301 机器人间交互数据
typedef __packed struct { 
	uint16_t data_cmd_id; /* 数据段的内容ID */
	uint16_t send_ID; /* 发送者的ID 若红1， 则ID为 1*/
	uint16_t receiver_ID; /* 接受者的ID */
}ext_student_interactive_header_data_t;

typedef __packed struct{
	float data1; /* 自定义浮点数据 1 */ 
	float data2; /* 自定义浮点数据 2 */
	float data3; /* 自定义浮点数据 3 */
	uint8_t masks; /* 自定义八位数据 位0-5 分别控制数据面板的六个指示灯 */
} client_custom_data_t;

//机器人之间通信数据 
/* 发送机器人之间通信信息时， 内容ID 为0x0200-0x02FF 
															发送者ID 为发送机器人的ID 
															接受者ID 为接受机器人的ID 
															数据段的字节数要小于113 */
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;
extern	moto_param    moto_Rpit;
extern	moto_param    moto_pit;
extern	moto_param    moto_yaw;
extern	moto_param    moto_chassis[6];
extern   gyro_param     gyro_pit;
extern   moto_param    moto_stir;
extern   moto_param    moto_bullet;
extern   moto_param    moto_fric[2];
extern   gyro_param     gyro_yaw;
extern   wl4data           data4bytes;
extern   float capvot;

extern ext_game_state_t               ext_game_state;
extern ext_game_robot_state_t     ext_game_robot_state;
extern ext_power_heat_data_t      ext_power_heat_data;
extern ext_game_robot_pos_t       ext_game_robot_pos;
extern ext_buff_musk_t                 ext_buff_musk;
extern ext_shoot_data_t                ext_shoot_data;
extern ext_robot_hurt_t                 ext_robot_hurt;

void can_device_init(void);
void can_receive_start(void);
void send_gimbal_ms(uint32_t id,uint8_t data[8]);
void send_chassis_ms(uint32_t id,uint8_t data[8]);
void encoder_data_handle(moto_param* ptr,uint8_t RxData[8]);
void gyro_data_handle(wl2data* ptr,wl4data* ptrr,gyro_param* gyro,uint8_t RxData[8]);
void send_gimbal_cur(uint32_t id,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);   //CAN 2
void send_chassis_cur(uint32_t id,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);  //CAN 1
void super_data_handle(wl4data* ptr1,uint8_t RxData[8]);
void send_405(uint32_t id,uint8_t data[8]);
#endif
