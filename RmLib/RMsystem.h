#ifndef __RMSYSTEM_H__
#define __RMSYSTEM_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"


#define BEEP_ERR
#ifdef BEEP_ERR
  #define BEEP_TUNE TIM3->ARR
  #define BEEP_CTRL TIM3->CCR1
#endif
#define DEFAULT_TUNE  300



/* can relevant */





/* pid debug relevant*/ 

// only one "1" can be exist a time, if not, GG.
/* pid relevant*/ 

#define DETECT_FLOW_LED_ON(i) flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i)

/* shoot relevant */

#define SNAIL_ON                     1350
#define SNAIL_OFF                    1000



#endif

