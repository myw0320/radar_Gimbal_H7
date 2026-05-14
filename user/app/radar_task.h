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
#include "packet_typedef.h"
#define RADAR_UART huart10
#define UART10_RX_LEN 36u
#define RADAR_RX_LEN  18u

typedef struct
{
    uint8_t robot_id;

}referee_receive_packet_t;
/******** 雷达数据包 ********/

typedef struct __attribute__((packed))
{
    uint8_t sod;
    uint8_t header;
    uint8_t reserved;
    float yaw;
    float pitch;
    float latency_time;
    uint16_t check_crc16;
    uint8_t end;

    packet_state_e packet_state;
} radar_receive_packet_t;
//
// typedef struct __attribute__((packed))
// {
//     uint8_t header;
//     uint8_t reserved;
//     float yaw;
//     float pitch;
//     float roll;
//     uint16_t checksum;
// } radar_transmit_packet_t;
typedef struct
{
    radar_receive_packet_t receive_packet;
    //vision_transmit_packet_t transmit_packet;

}radar_data_t;

void radar_init(void);
#endif //RADAR_GIMBAL_H7_RADAR_TASK_H