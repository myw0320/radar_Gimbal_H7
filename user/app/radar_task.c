#include "radar_task.h"


uint8_t radar_rx_buf[2][RADAR_RX_LEN];//

radar_data_t radarData;

void radar_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&RADAR_UART, (uint32_t *)radar_rx_buf[0], (uint32_t *)radar_rx_buf[1], RADAR_RX_LEN);
}

void Radar_GetRxPacket(const uint8_t *rx_data,radar_receive_packet_t *packet)
{
    if (rx_data[0] == 0xA5 && rx_data[2] == 0x00 && rx_data[21] == 0xFE)
    {
        packet->header = rx_data[1];

        memcpy(&(packet->yaw), &rx_data[3], 4);

        memcpy(&(packet->pitch), &rx_data[7], 4);

        memcpy(&(packet->current_receive_time), &rx_data[15], 4);

        packet->check_crc16 = (uint16_t)(rx_data[20] << 8 | rx_data[19]);

    }
}

void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {

        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx,RADAR_RX_LEN);

        /* Juge whether size is equal to the length of the received data */
        if(Size == RADAR_RX_LEN)
        {
            Radar_GetRxPacket(radar_rx_buf[0],&radarData.receive_packet);
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
        __HAL_DMA_SET_COUNTER(huart->hdmarx,RADAR_RX_LEN);

        if(Size == RADAR_RX_LEN)
        {
            Radar_GetRxPacket(radar_rx_buf[1],&radarData.receive_packet);
        }
    }
}