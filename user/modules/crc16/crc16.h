//
// Created by myw04 on 2026/1/19.
//

#ifndef RADAR_GIMBAL_H7_CRC16_H
#define RADAR_GIMBAL_H7_CRC16_H
#include "main.h"

uint16_t CRC16_Check(const uint8_t *data, uint16_t len);
uint8_t Verify_CRC16(const uint8_t *data, uint16_t len, uint16_t checksum);

#endif //RADAR_GIMBAL_H7_CRC16_H

