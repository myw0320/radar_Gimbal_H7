
#ifndef __H7_CAN_COMM_TASK_H
#define __H7_CAN_COMM_TASK_H
#include "main.h"
#include "fdcan.h"


#define YAW_CAN hfdcan1
#define YAW_CAN_ID 0x01

#define PITCH_CAN hfdcan2
#define PITCH_CAN_ID 0x02
void can_tx_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *tx_data);
#endif //__H7_CAN_COMM_TASK_H