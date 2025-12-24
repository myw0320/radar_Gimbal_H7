#include "bsp_can.h"

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
  FDCAN_FilterTypeDef filterConfig;
  /* Configure Rx filter */
  filterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filterConfig.FilterID1 = 0x00000000; //
  filterConfig.FilterID2 = 0x00000000; //
  if(HAL_FDCAN_ConfigFilter(&hfdcan2, &filterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 全局过滤设置 */
  /* 接收到消息ID与标准ID过滤不匹配，不接受 */
  /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
  /* 过滤标准ID远程帧 */
  /* 过滤扩展ID远程帧 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  /* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}




