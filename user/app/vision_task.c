#include "vision_task.h"
#include "usart.h"


uint8_t vision_rx_buf[2][UART7_RX_LEN];//视觉数据包

vision_data_t visionData;


void vision_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&VISION_UART,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],UART7_RX_LEN);
}

static uint16_t vision_calc_checksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum;
}

void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet)
{
    if (packet->header == 0xA5)
    {
        memcpy(&packet, rx_data, sizeof(vision_receive_packet_t));
    }

}

void Vision_AddTxPacket(uint8_t *tx_data,vision_transmit_packet_t *pactet)
{
    memcpy(&tx_data, pactet, sizeof(vision_transmit_packet_t));
}

void Vision_SendData(vision_data_t *send,float yaw,
    float pitch,float roll)
{
    static uint8_t tx_buf[8];
    send->transmit_packet.header = 0x5A;
    //0:red;1:blue
    send->transmit_packet.yaw = yaw;
    send->transmit_packet.pitch = pitch;
    send->transmit_packet.roll = 0;
    Vision_AddTxPacket (tx_buf,&send->transmit_packet);
    //发送
    HAL_UART_Transmit_DMA(&VISION_UART,(uint8_t *)tx_buf,sizeof(tx_buf));
}


void USER_USART7_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    static uint16_t this_time_rx_len = 0;
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART7_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART7_RX_LEN;
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

        /* Juge whether size is equal to the length of the received data */
        if(this_time_rx_len == VISION_RX_LEN)
        {
            Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);
        }

    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART7_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART7_RX_LEN;
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

         if(this_time_rx_len == VISION_RX_LEN)
         {
             Vision_GetRxPacket(vision_rx_buf[1],&visionData.receive_packet);
         }
    }
}
