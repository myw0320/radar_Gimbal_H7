#include "fsi6.h"


uint8_t fsi6_rx_buf[2][SBUS_FS_RX_LEN];//¿ØÊý¾Ý°ü
fsi6_data_struct fsi6Data;

void Fsi6_Init(void)
{
	USART_RxDMA_MultiBuffer_Init(&huart9,(uint32_t *)fsi6_rx_buf[0],(uint32_t *)fsi6_rx_buf[1],SBUS_FS_RX_LEN);
}


void Fsi6_RxPacketUpdate(volatile const uint8_t *sbus_buf, fsi6_data_struct *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
	if(sbus_buf[0] == 0x0f && sbus_buf[23] == 0x00)
	{
		rc_ctrl-> SBUS .CH[1] =(sbus_buf[1]>>0	  |( sbus_buf[2]<<8)) & 0x07ff;
		rc_ctrl-> SBUS .CH[2] =(sbus_buf[2]>>3	  | (sbus_buf[3]<<5)) & 0x07ff;
		rc_ctrl-> SBUS .CH[3] =((sbus_buf[3]>>6)  |(sbus_buf[4]<<2) | (sbus_buf[5]<<10)) & 0x07ff;
		rc_ctrl-> SBUS .CH[4] =((sbus_buf[5]>>1)  | (sbus_buf[6]<<7)) & 0x07ff;
		rc_ctrl-> SBUS .CH[5] =((sbus_buf[6]>>4)  | (sbus_buf[7]<<4)) & 0x07ff;
		rc_ctrl-> SBUS .CH[6] =((sbus_buf[7]>>7)  | (sbus_buf[8]<<1) | (sbus_buf[9]<<9)) & 0x07ff;
		rc_ctrl-> SBUS .CH[7] =((sbus_buf[9]>>2)  | (sbus_buf[10]<<6)) & 0x07ff;
		rc_ctrl-> SBUS .CH[8] =((sbus_buf[10]>>5) | (sbus_buf[11]<<3)) & 0x07ff;
		rc_ctrl-> SBUS .CH[9] =((sbus_buf[12]<<0) | (sbus_buf[13]<<8)) & 0x07ff;
		rc_ctrl-> SBUS .CH[10]=((sbus_buf[13]>>3) | (sbus_buf[14]<<5)) & 0x07ff;
		rc_ctrl-> SBUS .CH[11]=((sbus_buf[14]>>6) | (sbus_buf[15]<<2)|(sbus_buf[16]<<10) )& 0x07ff;
		rc_ctrl-> SBUS .CH[12] =((sbus_buf[16]>>1)| (sbus_buf[17]<<7)) & 0x07ff;
		rc_ctrl-> SBUS .CH[13] =((sbus_buf[17]>>4)| (sbus_buf[18]<<4)) & 0x07ff;
		rc_ctrl-> SBUS .CH[14] =((sbus_buf[18]>>7)| (sbus_buf[19]<<1)|(sbus_buf[20]<<9))& 0x07ff;
		rc_ctrl-> SBUS .CH[15]=((sbus_buf[20]>>2) | (sbus_buf[21]<<6)) & 0x07ff;
		rc_ctrl->rc.ch[0] = (int16_t)(rc_ctrl-> SBUS .CH[1] - 1024);
		rc_ctrl->rc.ch[1] = (int16_t)(rc_ctrl-> SBUS .CH[2] - 1024);
		rc_ctrl->rc.ch[2] = (int16_t)(rc_ctrl-> SBUS .CH[3] - 1024);
		rc_ctrl->rc.ch[3] = (int16_t)(rc_ctrl-> SBUS .CH[4] - 1024);
		rc_ctrl->rc.rotory_sw[0] = (rc_ctrl-> SBUS .CH[5] - 240);
		rc_ctrl->rc.rotory_sw[1] = (rc_ctrl-> SBUS .CH[6] - 240);
		switch(rc_ctrl-> SBUS .CH[7])
		{
			case 240:
				rc_ctrl->rc.sw[0] = 0;
			break;

			case 1807:
				rc_ctrl->rc.sw[0] = 1;
			break;
		}
		switch(rc_ctrl-> SBUS .CH[8])
		{
			case 240:
				rc_ctrl->rc.sw[1] = 0;
			break;

			case 1807:
				rc_ctrl->rc.sw[1] = 1;
			break;
		}
		switch(rc_ctrl-> SBUS .CH[9])
		{
			case 240:
				rc_ctrl->rc.sw[2] = 0;
			break;

			case 1807:
				rc_ctrl->rc.sw[2] = 1;
			break;
			case 1024:
				rc_ctrl->rc.sw[2] = 2;
			break;
		}
		switch(rc_ctrl-> SBUS .CH[10])
		{
			case 240:
				rc_ctrl->rc.sw[3] = 0;
			break;

			case 1807:
				rc_ctrl->rc.sw[3] = 1;
			break;
		}
	}
}

//
void USER_USART9_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
	/* Current memory buffer used is Memory 0 */
	if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
	{

		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 0 to Memory 1*/
		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_FS_RX_LEN);

		/* Juge whether size is equal to the length of the received data */
		// if(Size == REMOTE_RX_LEN)
		// {

			/* Memory 0 data update to remote_ctrl*/
			Fsi6_RxPacketUpdate(fsi6_rx_buf[0],&fsi6Data);

		// }

	}
	/* Current memory buffer used is Memory 1 */
	else
	{
		/* Disable DMA */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* Switch Memory 1 to Memory 0*/
		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		/* Reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_FS_RX_LEN);

		// if(Size == REMOTE_RX_LEN)
		// {
			/* Memory 1 to data update to remote_ctrl*/
			Fsi6_RxPacketUpdate(fsi6_rx_buf[1],&fsi6Data);
		// }

	}

}