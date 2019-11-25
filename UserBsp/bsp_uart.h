
/** @file bsp_uart.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief uart receive data from DBus/judge_system/manifold etc.
 *
 *
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
 #include <stdio.h>
#define UART_RX_DMA_SIZE       1024
/* usart relevant */
#define Dbus_usart              huart1 //for dji remote controler reciever

#define BT_usart                 huart6 //for debug 
#define DBUS_USART         USART1 //for dji remote controler reciever

#define BT_USART              USART6 //for debug
void uart1_device_init(void);
void uart7_device_init(void);
void uart6_device_init(void);
void computer_uart_init(void);
void judgement_uart_init(void);

void uart_receive_handler(UART_HandleTypeDef *huart);
void 	UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);

extern  uint8_t uart7_buff[50],uart6_buff[50],uart1_buff[50];
#endif
