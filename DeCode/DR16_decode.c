
/** @file DR16_decode.c
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief remote control message handle
 *
 */
#include "DR16_decode.h"
#include "string.h"
#include "usart.h"
#include "bsp_uart.h"
rc_info_t   rc;

uint8_t  dbus_buf[DBbus_BUFLEN];
 
void rc_callback_handle(rc_info_t *rc, uint8_t *buff)
{
	if(buff[12] < 0x02 && buff[13] < 0x02)
	{
	  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
	  rc->ch0 -= 1024;
	  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	  rc->ch1 -= 1024;
	  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	  rc->ch2 -= 1024;
	  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	  rc->ch3 -= 1024;
	  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	  rc->sw2 = (buff[5] >> 4) & 0x0003;	  
		rc->ch1 =	abs(rc->ch1) > 10 ? rc->ch1 : 0;
		rc->ch2 =	abs(rc->ch2) > 10 ? rc->ch2 : 0;
		rc->ch3 =	abs(rc->ch3) > 10 ? rc->ch3 : 0;
		rc->ch0 =	abs(rc->ch0) > 10 ? rc->ch0 : 0;
		  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
		  rc->mouse.y = buff[8] | (buff[9] << 8);
		  rc->mouse.z = buff[10] | (buff[11] << 8);		  
		  rc->mouse.l = buff[12];
		  rc->mouse.r = buff[13];
	      rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
  }	
	__HAL_UART_CLEAR_PEFLAG(&Dbus_usart);
}
/**
 * @brief Reset DBUS value
 * @param DBUS_Typedef
 * @return None
 */
void rc_init(void){
	memset(&rc,0,sizeof(rc_info_t));
  rc.sw1 = RC_DN;
  rc.sw2 = RC_DN;
}

