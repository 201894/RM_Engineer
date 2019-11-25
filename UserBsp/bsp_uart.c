/** @file bsp_uart.c
 *  @version 4.0
 *  @date  June 2019
 *
 *  @brief receive uart message and send it to usart2 ,deal with the message in STMGood
 */
#include "bsp_uart.h"
#include "STMGood.h"
//#include "detect_task.h"
#include "DR16_decode.h"
#include "usart.h"
#include "stdio.h"
//#include "Vision_decode.h"

/* dma double buffer */
uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
uint8_t pc_dma_rxbuff[2][UART_RX_DMA_SIZE];
uint8_t uart7_buff[50],uart6_buff[50],uart3_buff[50];

/**
  * @brief   initialize uart device 
  */
void uart1_device_init(void)
{
	HAL_DMA_Start_IT(&hdma_usart1_rx,(uint32_t)Dbus_usart.Instance->DR,(uint32_t)dbus_buf,DBbus_BUFLEN);
	huart1.Instance->CR3 |= USART_CR3_DMAR;
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,dbus_buf,DBbus_BUFLEN);	
}


void uart6_device_init(void)
{	
	HAL_DMA_Start_IT(&hdma_usart6_rx,(uint32_t)BT_usart.Instance->DR,(uint32_t)uart6_buff,1);
	BT_usart.Instance->CR3 |= USART_CR3_DMAR;
	__HAL_UART_ENABLE_IT(&BT_usart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&BT_usart,uart6_buff,1);  
}
/**
 * @brief Error Callback function
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //清除错误标志位，清空SR、DR寄存器
	}
}
int fputc(int ch, FILE *f)
{ 	
	while((BT_USART->SR&0X40)==0); 
	BT_USART->DR = (uint8_t) ch;      
	return ch;
}
/**
 * @brief uart Interrupt function
 * @param None
 * @return None
 * @attention Replace huart1 interrupt in stm32f4xx_it.c
 */
void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart){
	if(huart->Instance == DBUS_USART)
	{
		if(__HAL_UART_GET_FLAG(&Dbus_usart,UART_FLAG_IDLE) != RESET){
			__HAL_UART_CLEAR_IDLEFLAG(&Dbus_usart);		
			HAL_UART_DMAStop(&Dbus_usart);
			HAL_UART_Receive_DMA(&Dbus_usart,dbus_buf,DBbus_BUFLEN);

		}
	}
	if(huart->Instance == BT_USART)
	{
			if(__HAL_UART_GET_FLAG(&BT_usart,UART_FLAG_IDLE) != RESET){
			__HAL_UART_CLEAR_IDLEFLAG(&BT_usart);		
			HAL_UART_DMAStop(&BT_usart);
			HAL_UART_Receive_DMA(&BT_usart,uart7_buff,1);
		}
	}
}
	


