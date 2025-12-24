//
// Created by myw04 on 2025/12/11.
//

#ifndef __CAN_COMM_H
#define __CAN_COMM_H
#include "main.h"
#include "fdcan.h"
#include "Dji_motor.h"
#include "DM_motor.h"
HAL_StatusTypeDef can_tx_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *tx_data, uint32_t len);
#endif //__CAN_COMM_H