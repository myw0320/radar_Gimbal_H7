#include "bsp_usart.h"
#include "string.h"
#include "usart.h"


//dma接收初始化
void uart_dma_rx_init(UART_HandleTypeDef *huart,uint8_t *rx1_buf, uint8_t *rx2_buf, uint32_t buf_len)
{
    //使能UART DMA接收
    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
    //启用空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    //失能DMA
    __HAL_DMA_DISABLE(huart->hdmarx);

    ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->M0AR = (uint32_t)rx1_buf;
    //内存缓冲区2
    ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->M1AR = (uint32_t)rx2_buf;
    //数据长度
    ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->NDTR = buf_len;
    //使能双缓冲区
    ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->CR |= DMA_SxCR_DBM;
    //使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}
//弱定义数据处理函数
__weak void uart_rx_date_treating(uint8_t *data)
{

}
//串口dma回调
void UART_RxIRQHandler(UART_HandleTypeDef *huart,uint16_t len)
{
    if (huart->Instance->ISR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);//清除空闲中断
    }
    else if (huart->Instance->ISR & UART_FLAG_IDLE)//检测到空闲
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(huart);//清除空闲中断

        if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
        {
            //失能DMA
            __HAL_DMA_DISABLE(huart->hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = len - ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->NDTR;
            //重新设定数据长度
            ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->NDTR = len;
            //设定缓冲区1
            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            if(this_time_rx_len == len >> 1)
            {
                //接受数据
                if (huart == &huart1)
                {

                }
                else if (huart == &huart7)
                {

                }
                else if (huart == &huart9)
                {

                }
            }
        }
        else
        {
            //失能DMA
            __HAL_DMA_DISABLE(huart->hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = len - ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->NDTR;
            //重新设定数据长度
            ((DMA_Stream_TypeDef   *)huart->hdmarx->Instance)->NDTR = len;
            //设定缓冲区0
            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);

            if(this_time_rx_len == len >> 1)
            {
                if (huart == &huart1)
                {

                }
                else if (huart == &huart7)
                {

                }
                else if (huart == &huart9)
                {

                }
            }
        }
    }
}


