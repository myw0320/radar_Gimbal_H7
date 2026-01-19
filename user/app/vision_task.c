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
    if (rx_data[0] == 0xA5)//检查包头
    {
        // 2. 解析目标 X 坐标 (float, 占用偏移 2, 3, 4, 5)
        memcpy(&(packet->x), &rx_data[2], 4);
        packet->x_error = packet->x - X_OFFSET;

        // 3. 解析目标 Y 坐标 (float, 占用偏移 6, 7, 8, 9)
        memcpy(&(packet->y), &rx_data[6], 4);
        packet->y_error = packet->y - Y_OFFSET;

        // 4. 解析延迟时间 (float, 占用偏移 10, 11, 12, 13)
        memcpy(&(packet->current_receive_time), &rx_data[10], 4);

        // 5. 解析 CRC16 校验和 (uint16_t, 偏移 14, 15)
        //低字节在前 (rx_data[14])，高字节在后 (rx_data[15])
        packet->check_crc16 = (uint16_t)(rx_data[15] << 8 | rx_data[14]);
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
             Vision_GetRxPacket(vision_rx_buf[1],&visionData.receive_packet);
         }
    }
}
