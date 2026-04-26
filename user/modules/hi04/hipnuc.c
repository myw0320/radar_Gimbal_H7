//
// Created by myw04 on 2026/2/9.
//

#include "hipnuc.h"

uint8_t hi04_rx_buf[2][UART8_RX_LEN];
uint8_t *current_buffer;
uint16_t uart_rx_index = 0;




void hi04_init(hi04_imu_data_t *init)
{
    USART_RxDMA_MultiBuffer_Init(&huart8,(uint32_t*)hi04_rx_buf[0],(uint32_t*)hi04_rx_buf[1],UART8_RX_LEN);
    init->hipnuc_date = &hipnuc_raw.hi91;
}


void Hi04_GetRxPacket(hi04_imu_data_t *packet)
{
    for (uint16_t i = 0; i < packet->uart_rx_index; i++)
    {
        if (hipnuc_input(&hipnuc_raw, current_buffer[i]))
        {
            packet->ok_flag = 1;

        }
        else
        {
            packet->ok_flag = 0;
        }
    }
}

void USER_USART8_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    static uint16_t this_time_rx_len = 0;
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART8_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART8_RX_LEN;
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

        current_buffer = hi04_rx_buf[0];
        uart_rx_index = this_time_rx_len;
        /* Juge whether size is equal to the length of the received data */
        // if(this_time_rx_len == HI04_RX_LEN)
        // {
        //     //Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);
        // }

    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART8_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART8_RX_LEN;
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

        current_buffer = hi04_rx_buf[1];
        uart_rx_index = this_time_rx_len;
        // if(this_time_rx_len == HI04_RX_LEN)
        // {
            //Vision_GetRxPacket(vision_rx_buf[1],&visionData.receive_packet);
        // }
    }
}