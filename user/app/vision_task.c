#include "vision_task.h"

#include "usart.h"


uint8_t vision_rx_buf[2][VISION_RX_LEN];//视觉数据包

vision_data_t visionData;


void vision_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&huart7,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],VISION_RX_LEN);
}

void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet)
{
    if (rx_data[0] == 0xAA)//检查包头
    {
        packet->x = (float)(rx_data[1]<<8 | rx_data[2])/10.0f;
        packet->y = (float)(rx_data[3]<<8 | rx_data[4])/10.0f;
    }
}

// void vision_to_gimbal(float *yaw,float *pitch,vision_receive_packet_t *gimbal_control)
// {
//
// }

void USER_USART7_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx,VISION_RX_LEN);

        /* Juge whether size is equal to the length of the received data */
        if(Size == VISION_RX_LEN)
        {
            Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);
        }

    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx,VISION_RX_LEN);

         if(Size == VISION_RX_LEN)
         {
             Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);
         }
    }
}
