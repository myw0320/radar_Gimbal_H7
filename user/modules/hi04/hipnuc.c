//
// Created by myw04 on 2026/2/9.
//

#include "hipnuc.h"

uint8_t hi04_rx_buf[2][UART8_RX_LEN];

void hi04_init()
{
    //USART_RxDMA_MultiBuffer_Init(&HI04_UART,(uint32_t*)hi04_rx_buf[0],(uint32_t*)hi04_rx_buf[1],UART8_RX_LEN);
}


void Hi04_GetRxPacket()
{

}
//
// void USER_USART8_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
// {
//     static uint16_t this_time_rx_len = 0;
//     /* Current memory buffer used is Memory 0 */
//     if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
//     {
//         /* Disable DMA */
//         __HAL_DMA_DISABLE(huart->hdmarx);
//         this_time_rx_len = UART7_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART7_RX_LEN;
//         /* Switch Memory 0 to Memory 1*/
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
//
//         /* Reset the receive count */
//         __HAL_DMA_ENABLE(huart->hdmarx);
//
//         /* Juge whether size is equal to the length of the received data */
//         if(this_time_rx_len == VISION_RX_LEN)
//         {
//             Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);
//         }
//
//     }
//     /* Current memory buffer used is Memory 1 */
//     else
//     {
//         /* Disable DMA */
//         __HAL_DMA_DISABLE(huart->hdmarx);
//         this_time_rx_len = UART7_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART7_RX_LEN;
//         /* Switch Memory 0 to Memory 1*/
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
//
//         /* Reset the receive count */
//         __HAL_DMA_ENABLE(huart->hdmarx);
//
//         if(this_time_rx_len == VISION_RX_LEN)
//         {
//             Vision_GetRxPacket(vision_rx_buf[1],&visionData.receive_packet);
//         }
//     }
// }