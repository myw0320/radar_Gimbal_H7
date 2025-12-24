#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "stdint.h"
#include "usart.h"

void uart_dma_rx_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *huart_dma_rx, uint8_t* rx1_buf, uint8_t* rx2_buf, uint32_t buf_len);
void UART_RxIRQHandler(UART_HandleTypeDef* huart, uint16_t len);




#endif //__BSP_USART_H
