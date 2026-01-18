//
// Created by myw04 on 2025/12/26.
//

#ifndef RADAR_GIMBAL_H7_RADAR_TASK_H
#define RADAR_GIMBAL_H7_RADAR_TASK_H
#include "main.h"
#include "bsp_uart.h"
#define RADAR_RX_LEN  28u


/******** 雷达数据包 ********/
typedef struct __attribute__((packed))
{
    uint8_t header;//包头
    float x;
    float y;
    float z;
    float current_receive_time;//两次间隔时间
    uint16_t check_crc16;
}radar_receive_packet_t;//像素点数据

typedef struct
{
    radar_receive_packet_t receive_packet;
}radar_data_t;

void radar_init(void);
#endif //RADAR_GIMBAL_H7_RADAR_TASK_H