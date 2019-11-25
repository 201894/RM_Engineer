
/** @file DR16_decode.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief remote control message handle
 *
 */
 
#ifndef __DR16_DECODE_H__
#define __DR16_DECODE_H__

#include "stm32f4xx_hal.h"
#include "RMsystem.h"
#include "string.h"

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
};

/** 
  * @brief  remote control information
  */
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
  } kb;
} rc_info_t;
void rc_init(void);
#define Dbus_Max_Len       50
#define DBbus_BUFLEN     18
void      rc_callback_handle(rc_info_t *rc, uint8_t *buff);
extern  rc_info_t   rc,src;
extern   uint8_t  dbus_buf[DBbus_BUFLEN];
#endif
