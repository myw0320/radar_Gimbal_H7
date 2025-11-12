#include "bsp_usart.h"
#include "string.h"
#include "usart.h"

uint8_t vision_rx_buf[VISION_RX_LEN];//视觉数据包
uint8_t radar_rx_buf[RADAR_RX_LEN];//雷达数据包
uint8_t referee_rx_buf[REFEREE_RX_LEN];//裁判数据包
uint8_t remote_rx_buf[2][REMOTE_RX_LEN];//控数据包


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{

}

void Radar_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    huart->RxXferSize = DataLength;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->RDR,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
}

void RADAR_Init(void)
{
   // Radar_Init(&huart10,vision_rx_buf[0],vision_rx_buf[1],VISION_RX_LEN_2);
}

void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        //__HAL_DMA_SET_COUNTER(huart->hdmarx,VISION_RX_LEN_2);

        if(Size == VISION_RX_LEN)
        {
            //vision_rx_decode(vision_rx_buf[0]);
        }


    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

       // __HAL_DMA_SET_COUNTER(huart->hdmarx,VISION_RX_LEN_2);

        if(Size == VISION_RX_LEN)
        {
            //vision_rx_decode(vision_rx_buf[1]);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);

}




void REFREE_Init(void)
{
    //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, refree_rx_buf, REFREE_RX_LEN);
}

HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t len)
{
    return HAL_UART_Transmit_DMA(huart, tx_data, len);
}

