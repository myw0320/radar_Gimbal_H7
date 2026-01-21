//
// Created by myw04 on 2025/12/26.
//

#ifndef RADAR_GIMBAL_H7_RADAR_TASK_H
#define RADAR_GIMBAL_H7_RADAR_TASK_H
#include "main.h"
#include "stdint.h"
#include "bsp_uart.h"
#include "pid.h"
#include "arm_math.h"

#define RADAR_RX_LEN  28u

/******** 雷达数据包 ********/
typedef struct __attribute__((packed))
{
    uint8_t header;//包头
    float yaw;
    float pitch;
    float current_receive_time;//两次间隔时间
    uint16_t check_crc16;

    // float yaw_error;
    // float pitch_error;

}radar_receive_packet_t;//像素点数据

typedef struct
{
    radar_receive_packet_t receive_packet;
    pid_struct yaw_err_pid;
    pid_struct pitch_err_pid;
}radar_data_t;

void radar_init(void);
#endif //RADAR_GIMBAL_H7_RADAR_TASK_H