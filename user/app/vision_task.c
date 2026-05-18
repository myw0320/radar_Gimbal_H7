#include "vision_task.h"
#include "detect_task.h"
#include "usart.h"
#include "crc16.h"
#include <math.h>
#include <string.h>

uint8_t vision_rx_buf[2][UART7_RX_LEN];//视觉数据包

vision_data_t visionData;


void Vision_Init(void)
{
    USART_RxDMA_MultiBuffer_Init(&VISION_UART,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],UART7_RX_LEN);
}

static void Vision_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)(value >> 8);
}

static uint8_t Vision_HexChar(uint8_t value)
{
    value &= 0x0Fu;
    return value < 10u ? (uint8_t)('0' + value) : (uint8_t)('A' + value - 10u);
}

void Vision_GetRxPacket(const uint8_t *rx_data, vision_receive_packet_t *packet)
{
    uint16_t rx_crc = 0;
    uint16_t calc_crc = 0;

    if (rx_data == NULL || packet == NULL)
    {
        return;
    }

    if (rx_data[0] == 0xA5)
    {
        packet->header = rx_data[0];
        packet->reserved = rx_data[1];

        memcpy(&(packet->raw_yaw), &rx_data[2], sizeof(packet->raw_yaw));

        memcpy(&(packet->raw_pitch), &rx_data[6], sizeof(packet->raw_pitch));

        memcpy(&(packet->latency_time), &rx_data[10], sizeof(packet->latency_time));

        //packet->check_sum = rx_crc;

        detect_hook(VISION_TOE);
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

            packet->receive_time = HAL_GetTick() * 0.001f;
        }
    }
    else
    {
        packet->packet_state = DEC_ERROR;
    }
}

HAL_StatusTypeDef Vision_SendData(vision_data_t *send,float yaw,float pitch,float roll)
{
    static uint8_t tx_buf[VISION_TX_LEN];

    if (send == NULL)
    {
        return HAL_ERROR;
    }

    if (VISION_UART.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    send->transmit_packet.header = 0x5A;
    send->transmit_packet.reserved = 0;
    send->transmit_packet.yaw = yaw;
    send->transmit_packet.pitch = pitch;
    send->transmit_packet.roll = roll;

    append_CRC16_check_sum((uint8_t*)&send->transmit_packet, sizeof(send->transmit_packet) - 2);

    memcpy(tx_buf, &send->transmit_packet, sizeof(send->transmit_packet));
    //Vision_WriteU16Le(&tx_buf[VISION_CRC_OFFSET], send->transmit_packet.check_sum);
    return HAL_UART_Transmit_DMA(&VISION_UART, tx_buf, sizeof(tx_buf));
}

HAL_StatusTypeDef Vision_SendTestPacket(void)
{
    static uint8_t raw_buf[VISION_TX_LEN];
    static uint8_t tx_buf[VISION_TX_LEN * 3u + 1u];
    static uint16_t test_count = 0;
    float yaw = 1.2345f;
    float pitch = -2.3456f;
    float roll = 3.4567f;
    uint16_t crc = 0;

    if (VISION_UART.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    raw_buf[VISION_HEAD_OFFSET] = 0x5A;
    raw_buf[VISION_RESERVED_OFFSET] = (uint8_t)test_count;
    memcpy(&raw_buf[VISION_YAW_OFFSET], &yaw, sizeof(yaw));
    memcpy(&raw_buf[VISION_PITCH_OFFSET], &pitch, sizeof(pitch));
    memcpy(&raw_buf[VISION_LATENCY_OFFSET], &roll, sizeof(roll));

    crc = CRC16_Check(raw_buf, VISION_CRC_OFFSET);
    Vision_WriteU16Le(&raw_buf[VISION_CRC_OFFSET], crc);

    for (uint8_t i = 0; i < VISION_TX_LEN; i++)
    {
        tx_buf[i * 3u] = Vision_HexChar(raw_buf[i] >> 4);
        tx_buf[i * 3u + 1u] = Vision_HexChar(raw_buf[i]);
        tx_buf[i * 3u + 2u] = (i == (VISION_TX_LEN - 1u)) ? '\r' : ' ';
    }
    tx_buf[VISION_TX_LEN * 3u] = '\n';
    test_count++;

    return HAL_UART_Transmit_DMA(&VISION_UART, tx_buf, sizeof(tx_buf));
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
