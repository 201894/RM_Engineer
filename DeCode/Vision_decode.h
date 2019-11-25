


#ifndef _VISION_DECODE_H_
#define _VISION_DECODE_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#define VISIONDATALENGTH 26

//reference base aim
#define reference_centerX  510              
#define reference_centerY 435

typedef union{
	int16_t d;
	uint8_t c[2];
}int16uchar;

typedef union{
	float f;
	uint8_t c[4];
}float4uchar;

typedef union{
   float f;
	 unsigned char uc[4];
}float2uchar;


typedef struct {
	 uint8_t isTrue; 
	
	 float pcTargetX;//adjust value
	 float pcTargetY;
	
	 int refer_centerX;
	 int refer_centerY;
	
	 float2uchar pcCenterX;
	 float2uchar pcCenterY;
	 float2uchar pcCenterZ;
	
	 float2uchar pcCompensationX;
	 float2uchar pcCompensationY;
	
	 float CompXout;
	 float CompYout;
	
	 float adjustX;
	 float adjustY;
} pcDataParam;

typedef struct{
	portTickType time;
	portTickType timelast;
	unsigned char mode; // 1 auto 2 sudoku_number 3 sudoku_finddiff
	int16uchar pitch_angle;
	int16uchar yaw_angle;
	int16uchar yaw_speed;
	float4uchar shoot_speed;
	float4uchar distance;
}VisionData_t;

typedef struct{
	portTickType timecur;
	int32_t timepassed;
	int16_t yaw_front;
	int16_t pitch_front;
	float anglenow;
	float anglepoint;
	
	float cloud_anglepoint;
	float cloud_anglepoint_last;
	int16_t pitch_anglepoint;
	float yawspeed_point;
}VisionCalinfo_t;

extern pcDataParam pcParam,pcParamLast;
extern VisionData_t TargetData;
extern float sbs;

extern int flaglose;

void TargetTimeFresh(void);
void Vision_IRQ(void);
void pcDataInit(void);
void Vision_Decode(void);
void VisionInit(void);
void pc_data_ex(uint8_t mode,uint8_t number,int cycle);
void BulletCntTask(uint8_t cnt,int freq);
extern pcDataParam pcParam,pcParamLast;
extern float Z;
extern float YaSpdFiter;
#endif
