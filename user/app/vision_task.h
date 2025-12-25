
#ifndef RADAR_GIMBAL_H7_VISION_TASK_H
#define RADAR_GIMBAL_H7_VISION_TASK_H
#include "stdint.h"
#include "ins_task.h"
#include "bsp_uart.h"
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
}vision_data_struct;//像素点数据

/******** 步兵自瞄 ********/
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t mod;
    float roll;
    float pitch;
    float yaw;
    float vx;
    float vy;
    float wz;
    float fill1;
    uint8_t fill;
    uint8_t ender;
}vision_transmit_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t fire;
    float pitch;
    float yaw;
    float distance;
    uint8_t ender;
}vision_receive_packet_t;

typedef struct
{
    float Now_Velocity_X;            // 当前速度X
    float Now_Velocity_Y;            // 当前速度Y
    float Now_Omega;                 // 当前角速度
}vision_chassis_t;

typedef struct
{
    // 绝对角指针
    const INS_t* vision_angle_point;
    // 识别颜色
    uint8_t detect_armor_color;
    // id
    uint8_t id;
    // 串口发送缓冲区
    uint8_t tx_buf[50];
    // 串口接收缓冲区
    uint8_t rx_buf[50];
    // 发送给视觉结构体
    vision_transmit_packet_t vision_send_packet;
    // 接收到视觉结构体
    vision_receive_packet_t Rx_data;

    vision_chassis_t vision_chassis;

}vision_ctr_t;

extern vision_data_struct visionData;

void vision_to_gimbal(float *yaw,float *pitch,vision_data_struct *gimbal_control);
#endif //RADAR_GIMBAL_H7_VISION_TASK_H