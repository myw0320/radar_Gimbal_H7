
#ifndef __H7_CAN_COMM_TASK_H
#define __H7_CAN_COMM_TASK_H
#include "main.h"
#include "fdcan.h"

void can_tx_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *tx_data);
#endif //__H7_CAN_COMM_TASK_H