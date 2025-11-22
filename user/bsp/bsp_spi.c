#include "bsp_spi.h"

#include "spi.h"


void spi_dma_init(SPI_HandleTypeDef *hspi,uint32_t rx_buf,uint16_t len)
{
    hspi->Instance->CR1 |= (0x1UL << (1U));
    //hspi->Instance->CR2 |= SPI_CR2_TXDMAEN_Msk;
    //使能spi
    __HAL_SPI_ENABLE(hspi);
    //失能DMA
    __HAL_DMA_DISABLE(hspi->hdmarx);
    while (((DMA_Stream_TypeDef   *)hspi->hdmarx->Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_ENABLE(hspi->hdmarx);
    }
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,DMA_LISR_TCIF2);

    //((DMA_Stream_TypeDef   *)hspi->hdmarx->Instance)->PAR = (uint32_t) & (hspi->Instance-);
    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)hspi->hdmarx->Instance)->M0AR = (uint32_t)(rx_buf);

    //数据长度
    __HAL_DMA_SET_COUNTER(hspi->hdmarx,len);

    __HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC);
    //失能DMA
    __HAL_DMA_ENABLE(hspi->hdmarx);

}


void spi_dma_enable(SPI_HandleTypeDef *hspi,uint32_t tx_buf, uint32_t rx_buf,uint16_t len)
{
    __HAL_DMA_DISABLE(hspi->hdmarx);
    __HAL_DMA_DISABLE(hspi->hdmatx);

    while(((DMA_Stream_TypeDef   *)hspi->hdmarx->Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hspi->hdmarx);
    }
    while(((DMA_Stream_TypeDef   *)hspi->hdmatx->Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hspi->hdmatx);
    }

    __HAL_DMA_CLEAR_FLAG (hspi->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi->hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi->hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi->hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi->hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi->hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi->hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi->hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi->hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi->hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi->hdmatx));

    ((DMA_Stream_TypeDef   *)hspi->hdmarx->Instance)->M0AR = rx_buf;
    ((DMA_Stream_TypeDef   *)hspi->hdmatx->Instance)->M0AR = tx_buf;

    __HAL_DMA_SET_COUNTER(hspi->hdmarx, len);
    __HAL_DMA_SET_COUNTER(hspi->hdmatx, len);

    __HAL_DMA_ENABLE(hspi->hdmarx);
    __HAL_DMA_ENABLE(hspi->hdmatx);
}