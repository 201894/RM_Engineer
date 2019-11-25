
/** @file ramp.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief ramp contrl realization
 *
 *
 */

#ifndef __RAMP_H__
#define __RAMP_H__

#include "stm32f4xx_hal.h"

typedef struct
{
	int32_t count;
	int32_t scale;
	float out;
}ramp_t;

#define RAMP_DEFAULT_INIT \
{ \
	.count = 100, \
	.scale = 500, \
	.out = 0,\
};

#define RAMP_GEN_DAFAULT \
{ \
              .count = 0, \
              .scale = 0, \
              .out = 0, \
              .init = &ramp_init, \
              .calc = &ramp_calc, \
            } \

void  ramp_init(ramp_t *ramp, int32_t scale);
float ramp_calc(ramp_t *ramp);

#endif
						