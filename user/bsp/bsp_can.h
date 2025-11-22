//
// Created by myw04 on 2025/11/9.
//

#ifndef RADAR_GIMBAL_BSP_CAN_H
#define RADAR_GIMBAL_BSP_CAN_H
#include "stdint.h"
#include "fdcan.h"

void FDCAN1_Config_Init(void);
void FDCAN2_Config_Init(void);
HAL_StatusTypeDef can_tx_data(FDCAN_HandleTypeDef *hcan, uint8_t id, uint8_t *data, uint32_t len);
#endif //RADAR_GIMBAL_BSP_CAN_H