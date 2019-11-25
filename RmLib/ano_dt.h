
/** @file ano_dt.h
 *  @version 1.1
 *  @date  Mar 2019
 *
 *  @brief deal with data to draw a waveform
 *
 */
#ifndef  __ANO_DT_H_
#define __ANO_DT_H_

#include "stm32f4xx_hal.h"

typedef struct 
{
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t send_pid3;
		uint8_t send_pid4;
		uint8_t send_pid5;
		uint8_t send_pid6;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;

}dt_flag_t;

void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length);
void ANO_DT_Send_Senser(int16_t target,int16_t angle,float crtout,float speed,int16_t current,int16_t speed1,int16_t current1,int16_t speed2,int16_t current2,int32_t bar);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_Power(uint16_t votage, uint16_t current);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);

#endif