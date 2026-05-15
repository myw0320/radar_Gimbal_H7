#include "radar_task.h"

#include "detect_task.h"
#include <math.h>
#include <string.h>


uint8_t radar_rx_buf[2][UART10_RX_LEN];//

radar_data_t radarData;

void Radar_Init(void)
{
    USART_RxDMA_MultiBuffer_Init(&RADAR_UART, (uint32_t *)radar_rx_buf[0], (uint32_t *)radar_rx_buf[1], UART10_RX_LEN);
}

void Radar_GetRxPacket(const uint8_t *rx_data,radar_receive_packet_t *packet)
{
    if (rx_data[0] == 0xA5 && rx_data[1] == 0xA5 && rx_data[17] == 0xFE)
    {
        packet->sod = rx_data[0];

        packet->header = rx_data[1];

        memcpy(&(packet->raw_yaw), &rx_data[3], 4);

        memcpy(&(packet->raw_pitch), &rx_data[7], 4);

        memcpy(&(packet->latency_time), &rx_data[11], 4);

        //packet->check_crc16 = (uint16_t)(rx_data[15] << 8 | rx_data[16]);

        packet->end = rx_data[17];

        detect_hook(RADAR_TOE);
        packet->packet_state = DEC_OK;
        if (!isfinite(packet->raw_yaw) || !isfinite(packet->raw_pitch) ||
            packet->raw_yaw > 6.28f || packet->raw_pitch > 6.28f)
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

void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    static uint16_t this_time_rx_len = 0;
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {

        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART10_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART10_RX_LEN;
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

        /* Juge whether size is equal to the length of the received data */
        // if(Size == RADAR_RX_LEN)
        // {
            Radar_GetRxPacket(radar_rx_buf[0],&radarData.receive_packet);
        // }

    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        this_time_rx_len = UART10_RX_LEN - ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR;
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = UART10_RX_LEN;
        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        /* Reset the receive count */
        __HAL_DMA_ENABLE(huart->hdmarx);

        // if(Size == RADAR_RX_LEN)
        // {
            Radar_GetRxPacket(radar_rx_buf[1],&radarData.receive_packet);
        // }
    }
}
