/** @file bsp_io.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief basic IO port operation
 *
 *  HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
 */
#ifndef __BSP_IO_H__
#define __BSP_IO_H__
#include "stm32f4xx_hal.h"
#include  "gpio.h"
#define         trigger_1             HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1)
#define         trigger_2             HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
#define         trigger_3             HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)
#define         Light_Sensor      HAL_GPIO_ReadPin(LIGHT1_GPIO_Port,LIGHT1_Pin)

#define LED_G_ON   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_G_OFF HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_R_ON   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_R_OFF HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define led_init \
{\
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);\
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);\
}\
void pwm_device_init(void);
void sunnysky_init(void);
void busser_task(void);
void vcc_out_init(void);
#endif
