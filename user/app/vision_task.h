
#ifndef RADAR_GIMBAL_H7_VISION_TASK_H
#define RADAR_GIMBAL_H7_VISION_TASK_H
#include "stdint.h"
#include "ins_task.h"
#include "bsp_uart.h"
#include "pid.h"
#define VISION_RX_LEN  28u


/******** 雷达自瞄 ********/
typedef struct __attribute__((packed))
{
    uint8_t header;//包头
    float x;
    float y;
    float x_error;
    float y_error;

    float current_receive_time;//两次间隔时间
}vision_receive_packet_t;//像素点数据

typedef struct
{
    vision_receive_packet_t receive_packet;
    pid_struct x_err_pid;
    pid_struct y_err_pid;

}vision_data_t;

extern vision_data_t visionData;

void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet);
#endif //RADAR_GIMBAL_H7_VISION_TASK_H