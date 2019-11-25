/** @file km_handle.h
 *  @version 2.0
 *  @date 3.29 2019
 *
 *  @brief get rc information and handle it
 *
 *
 */

#ifndef __KM_HANDLE_H__
#define __KM_HANDLE_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"
/**********************************************************************************
 * bit      :      15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
 //#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 	0x0020
//#define Q 			0x0040
//#define E			0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B			0x8000
/******************************************************/
 //      speed      key


#define CAP_CTRL  (rc.kb.bit.SHIFT) // ≥¨º∂µÁ»› 
#define SL_SPD       (rc.kb.bit.CTRL)  // 
//      direction  key
#define UP                  (rc.kb.bit.W)
#define DOWN              (rc.kb.bit.S)
#define LEFT                  (rc.kb.bit.A)
#define RIGHT                 (rc.kb.bit.D)
#define GYRO_CTRL        (rc.kb.bit.X)
#define BULLET_CTRL       (rc.kb.bit.R)

#define KEY_V		    0x4000
#define KEY_C		    0x2000
#define KEY_X		    0x1000
#define KEY_Z		    0x0800
#define KEY_G         0x0400
#define KEY_F		    0x0200
#define KEY_R		    0x0100
#define KEY_E		    0x0080
#define KEY_Q		    0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		    0x0008
#define KEY_A		    0x0004
#define KEY_S		    0x0002
#define KEY_W		    0x0001

 typedef enum
{
  KEY_RELEASE = 0,
  KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG,
} kb_state_e;

typedef struct
{  
  float vx;
  float vy;
  float vw;
  
  float pit_v;
  float yaw_v;	
	
  uint8_t buff_ctrl;
  uint8_t  rotate_flag;
  uint8_t  middle_flag;
  uint8_t  ratio_flag;	
  uint8_t  ramp_flag;	
  uint16_t lk_cnt;
  uint16_t rk_cnt;
	
  uint16_t wk_cnt;
  uint16_t ak_cnt;
	
  uint16_t sk_cnt;
  uint16_t dk_cnt;
	
  kb_state_e lk_sta;
  kb_state_e rk_sta;
  
  kb_state_e wk_sta;
  kb_state_e sk_sta;
  
  kb_state_e ak_sta;
  kb_state_e dk_sta;	
  
} kb_ctrl_t;
void km_global_ctrl(void);
void key_fsm(kb_state_e *sta, uint8_t key);

void km_global_init(void);
extern float chassis_para;
extern kb_ctrl_t km;
#endif 
