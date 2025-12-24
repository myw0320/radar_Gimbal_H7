#include "bsp_usart.h"
#include "string.h"
#include "usart.h"
/* USER CODE BEGIN Includes */
#include "vision_task.h"
// #include "radar_task.h"
#include "remote.h"
/* USER CODE END Includes */


/**
  * @brief  初始化UART的DMA接收
  * @param  huart: 指向UART句柄的指针
  * @param  rx1_buf: 第一个接收缓冲区
  * @param  rx2_buf: 第二个接收缓冲区
  * @param  buf_len: 缓冲区长度
  * @retval 无
  */
void uart_dma_rx_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *huart_dma_rx, uint8_t* rx1_buf, uint8_t* rx2_buf, uint32_t buf_len)
{
     // 使能UART DMA接收
     SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); // 使能UART DMA接收
     // 启用空闲中断
     //__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // 启用空闲中断
     // 失能DMA
     __HAL_DMA_DISABLE(huart->hdmarx); // 失能DMA

     // 设置DMA外设地址
     ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->PAR = (uint32_t) & (USART1->RDR); // 设置DMA外设地址
     // 设置内存缓冲区1
     ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->M0AR = (uint32_t)rx1_buf; // 设置内存缓冲区1
     // 设置内存缓冲区2
     ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->M1AR = (uint32_t)rx2_buf; // 设置内存缓冲区2
     // 设置数据长度
     ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->NDTR = buf_len; // 设置数据长度
     // 使能双缓冲区
     SET_BIT(((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
     // 使能DMA
      __HAL_DMA_ENABLE(huart->hdmarx); // 使能DMA

    // __HAL_UART_CLEAR_IDLEFLAG(huart);
    // SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    // //__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // 启用空闲中断


}

/**
  * @brief  串口DMA接收中断处理函数
  * @param  huart: 指向UART句柄的指针
  * @param  len: 缓冲区长度
  * @retval 无
  */

void UART_RxIRQHandler(UART_HandleTypeDef* huart, uint16_t len)
{

    if (UART5->ISR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (UART5->ISR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(huart);
        // 检查当前使用的DMA缓冲区
        if ((((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            // 失能DMA
            __HAL_DMA_DISABLE(huart->hdmarx);
            // 计算本次接收的数据长度
            //this_time_rx_len = len - ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->NDTR;
            // 重新设定数据长度
            ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->NDTR = len;
            // 切换到缓冲区1
            ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
            // 使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            // 根据不同的UART实例进行处理
            // if (huart == &huart1)
            // {
            //     if (this_time_rx_len == len >> 1)
            //     {
            //         // TODO: 处理huart1的数据
            //     }
            // }
            // else if (huart == &huart5)
            // {
            //     // TODO: 处理huart5的数据
            //     Remote_RxPacketUpdate(remote_rx_buf[0],&rcData);
            // }
        }
        else
        {
            // 失能DMA
            __HAL_DMA_DISABLE(huart->hdmarx);
            // 计算本次接收的数据长度
            //this_time_rx_len = len - ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->NDTR;
            // 重新设定数据长度
            ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->NDTR = len;
            // 切换到缓冲区0
            ((DMA_Stream_TypeDef*)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            // 使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            // 根据不同的UART实例进行处理
            // if (huart == &huart1)
            // {
            //     if (this_time_rx_len == len >> 1)
            //     {
            //         // TODO: 处理huart1的数据
            //     }
            // }
            // else if (huart == &huart5)
            // {
            //     // TODO: 处理huart5的数据
            //     Remote_RxPacketUpdate(remote_rx_buf[1],&rcData);
            // }
        }
    }
}


