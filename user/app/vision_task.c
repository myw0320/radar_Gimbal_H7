#include "vision_task.h"

#include "detect_task.h"
#include "usart.h"
#include "crc16.h"

uint8_t vision_rx_buf[2][UART7_RX_LEN];//视觉数据包

vision_data_t visionData;


void vision_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&VISION_UART,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],UART7_RX_LEN);
}

uint16_t calculate_crc16(uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for(uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}
static uint16_t vision_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet)
{
    if (rx_data[0] == 0xA5)
    {
        packet->header = rx_data[0];

        memcpy(&(packet->raw_yaw), &rx_data[2], 4);

        memcpy(&(packet->raw_pitch), &rx_data[6], 4);

        memcpy(&(packet->latency_time), &rx_data[10], 4);

        packet->check_sum = (uint16_t)(rx_data[20] << 8 | rx_data[19]);

        packet->packet_state = DEC_OK;
        detect_hook(VISION_TOE);
        if (packet->raw_yaw > 6.28f && packet->raw_pitch > 6.28f)
        {
            packet->packet_state = DEC_DATA_NO;
            packet->yaw = packet->last_yaw;
            packet->pitch = packet->last_pitch;
        }
        else
        {
            packet->yaw = packet->raw_yaw;
            packet->pitch = packet->raw_pitch;
            packet->last_yaw = packet->yaw;
            packet->last_pitch = packet->pitch;
        }

    }
}

void Vision_AddTxPacket(uint8_t *tx_data,vision_transmit_packet_t *packet)
{
    //memcpy(&tx_data, &packet, 16);
    // memcpy(&tx_data[0], &(packet->header), 4);
    //
    // memcpy(&tx_data[2], &(packet->header), 4);
    //
    // memcpy(&tx_data[2], &(packet->header), 4);
}

void Vision_SendData(vision_data_t *send,float yaw,float pitch,float roll)
{
    static uint8_t tx_buf[8];
    send->transmit_packet.header = 0x5A;
    send->transmit_packet.yaw = yaw;
    send->transmit_packet.pitch = pitch;
    send->transmit_packet.roll = 0;
    //
    send->transmit_packet.check_sum = vision_crc16((uint8_t*)&send->transmit_packet, sizeof(send->transmit_packet) - 2);
    // Vision_AddTxPacket (tx_buf,&send->transmit_packet);
    //发送
    HAL_UART_Transmit(&VISION_UART,(uint8_t *)&send->transmit_packet,sizeof(send->transmit_packet),HAL_MAX_DELAY);
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
        // if(this_time_rx_len == VISION_RX_LEN)
        // {
            Vision_GetRxPacket(vision_rx_buf[0],&visionData.receive_packet);

        // }

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

         // if(this_time_rx_len == VISION_RX_LEN)
         // {
             Vision_GetRxPacket(vision_rx_buf[1],&visionData.receive_packet);
         // }
    }
}
