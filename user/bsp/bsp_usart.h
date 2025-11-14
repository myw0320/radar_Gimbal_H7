#ifndef RADAR_GIMBAL_H7_BSP_USART_H
#define RADAR_GIMBAL_H7_BSP_USART_H
#include "stdint.h"
#include "usart.h"

void uart_dma_rx_init(UART_HandleTypeDef *huart,uint8_t *rx1_buf, uint8_t *rx2_buf, uint32_t buf_len);
void UART_RxIRQHandler(UART_HandleTypeDef *huart,uint16_t len);




#endif
