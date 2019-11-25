/** @file ramp.c
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief ramp contrl realization
 *
 *
 */
#include "ramp.h"


void  ramp_init(ramp_t *ramp, int32_t scale)
{
	ramp->out = 0;
    ramp->scale = scale;  	
}

float ramp_calc(ramp_t *ramp)
{
  if(ramp->scale <=0) return 0;
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;  
    ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;	
}
