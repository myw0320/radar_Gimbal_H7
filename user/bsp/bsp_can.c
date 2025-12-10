#include "bsp_can.h"
#include "gimbal_task.h"
#include "detect_task.h"
void FDCAN1_Config_Init(void)
{
  FDCAN_FilterTypeDef filterConfig;
  /* Configure Rx filter */
  filterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filterConfig.FilterID1 = 0x00000000; //
  filterConfig.FilterID2 = 0x00000000; //
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK)
	{
		Error_Handler();
	}

/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */
/* 过滤扩展ID远程帧 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  /* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}


HAL_StatusTypeDef can_tx_data(FDCAN_HandleTypeDef *hcan, uint8_t id, uint8_t *tx_data, uint32_t len)
{

	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;// CAN ID
    TxHeader.IdType =  FDCAN_STANDARD_ID ;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	switch (len)
	{
		case 8:
			TxHeader.DataLength = len<<16;
			break;
		case 12:
			TxHeader.DataLength =FDCAN_DLC_BYTES_12;
			break;
		case 16:
			TxHeader.DataLength =FDCAN_DLC_BYTES_16;
			break;
		case 20:
			TxHeader.DataLength =FDCAN_DLC_BYTES_20;
			break;
		case 24:
			TxHeader.DataLength =FDCAN_DLC_BYTES_24;
			break;
		case 48:
			TxHeader.DataLength =FDCAN_DLC_BYTES_48;
			break;
		case 64:
			TxHeader.DataLength =FDCAN_DLC_BYTES_64;
			break;
	}
  TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;//消息标记

  return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, tx_data);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      FDCAN_RxHeaderTypeDef RxHeader1;
      uint8_t rx_data[8];
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, rx_data);

      switch(RxHeader1.Identifier)
      {
      	case 0x301:
      	{
      		DM_GetRxPacket(&gimbalControl.yawMotor,rx_data);
      		detect_hook(GIMBAL_YAW_TOE);
      		break;
      	}
      	case 0x302:
      	{
      		DM_GetRxPacket(&gimbalControl.pitchMotor,rx_data);
      		detect_hook(GIMBAL_PITCH_TOE);
      		break;
      	}
      }
    }
  	else if (hfdcan->Instance == FDCAN2)
  	{
  		FDCAN_RxHeaderTypeDef RxHeader2;
  		uint8_t rx_data[8];
  		switch(RxHeader2.Identifier)
  		{

  		}
  	}
  	else if (hfdcan->Instance == FDCAN3)
  	{
  		FDCAN_RxHeaderTypeDef RxHeader3;
  		uint8_t rx_data[8];
  		switch(RxHeader3.Identifier)
  		{

  		}
  	}
  }
}


