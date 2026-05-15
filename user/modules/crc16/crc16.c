//
// Created by myw04 on 2026/1/19.
//

#include "crc16.h"

uint16_t CRC16_Check(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // 多项式
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 验证函数
uint8_t Verify_CRC16(const uint8_t *data, uint16_t len, uint16_t checksum)
{
    return CRC16_Check(data, len) == checksum;
}
