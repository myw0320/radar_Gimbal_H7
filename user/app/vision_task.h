
#ifndef RADAR_GIMBAL_H7_VISION_TASK_H
#define RADAR_GIMBAL_H7_VISION_TASK_H
#include "stdint.h"
#include "bsp_uart.h"
#include "pid.h"
#include "arm_math.h"
#include "packet_typedef.h"
#define VISION_UART huart7

#define UART7_RX_LEN 32u
#define VISION_RX_LEN 16u

#define X_OFFSET 512
#define Y_OFFSET 512



typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t reserved:1;
    float raw_yaw;
    float raw_pitch;
    float latency_time;
    uint16_t check_sum;

    packet_state_e packet_state;
    float yaw;
    float pitch;
    float last_yaw;
    float last_pitch;
} vision_receive_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t reserved:1;
    float yaw;
    float pitch;
    float roll;
    uint16_t check_sum;
} vision_transmit_packet_t;


typedef struct
{
    vision_receive_packet_t receive_packet;
    vision_transmit_packet_t transmit_packet;

}vision_data_t;

extern vision_data_t visionData;

void Vision_Init(void);
void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet);
void Vision_SendData(vision_data_t *send,float yaw,float pitch,float roll);
#endif //RADAR_GIMBAL_H7_VISION_TASK_H