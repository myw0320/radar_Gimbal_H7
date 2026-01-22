#include "vision_task.h"

#include <sys/_intsup.h>

#include "usart.h"


uint8_t vision_rx_buf[2][UART7_RX_LEN];//视觉数据包

vision_data_t visionData;


void vision_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&VISION_UART,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],UART7_RX_LEN);
}

void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet)
{
    if (rx_data[0] == 0xA5)//检查包头
    {
        //视觉开始计时
        packet->receive_time = HAL_GetTick()/100.0f;
        //解包数据
        memcpy(&(packet->header), &rx_data[0], 1);
        memcpy(&(packet->reserved), &rx_data[1], 1);
        packet->current_x = (int16_t)(rx_data[3] << 8 | rx_data[2]);
        packet->current_y = (int16_t)(rx_data[5] << 8 | rx_data[4]);
        //数据判断
        if (fabs(packet->current_x) < 7750)
        {
            packet->x = (float)packet->current_x/10.0f;
            packet->ok_flag = 1;
        }
        else
        {
            packet->x = 0.0f;
            packet->ok_flag = 0;
        }

        if (fabs(packet->current_y) < 7450)
        {
            packet->y = (float)packet->current_y/10.0f;
            packet->ok_flag = 1;
        }
        else
        {
            packet->y = 0.0f;
            packet->ok_flag = 0;
        }
        memcpy(&(packet->current_receive_time), &rx_data[10], 4);
        //CRC校验
        packet->check_crc16 = (uint16_t)(rx_data[12] << 8 | rx_data[11]);

    }
    else
    {
        packet->current_x = 0;
        packet->current_y = 0;
        packet->x = 0.0f;
        packet->y = 0.0f;
        packet->ok_flag = 0;
    }
}

void Vision_AddTxPacket(uint8_t *tx_data,vision_transmit_packet_t *pactet)
{
    tx_data[0] = pactet->header;
    tx_data[1] = pactet->target_colors;
    tx_data[2] = pactet->check_crc16;//低位
    tx_data[3] = pactet->check_crc16 >> 8;//高位
    /***未使用***/
}

void Vision_SendData(vision_data_t *send)
{
    static uint8_t tx_buf[8];
    send->transmit_packet.header = 0x5A;
    //0:red;1:blue
    send->transmit_packet.target_colors = 1;

    //send->transmit_packet.check_crc16 =

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
