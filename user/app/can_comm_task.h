
#ifndef __H7_CAN_COMM_TASK_H
#define __H7_CAN_COMM_TASK_H
#include "main.h"
#include "fdcan.h"

HAL_StatusTypeDef can_tx_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *tx_data, uint32_t len);
#endif //__H7_CAN_COMM_TASK_H