
#ifndef RADAR_GIMBAL_H7_VISION_TASK_H
#define RADAR_GIMBAL_H7_VISION_TASK_H
#include "stdint.h"
#include "ins_task.h"
#include "bsp_uart.h"
#include "pid.h"

#define VISION_UART huart7

#define UART7_RX_LEN 24u
#define VISION_RX_LEN 16u

#define X_OFFSET 512
#define Y_OFFSET 512
/******** �״����� ********/

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t reserved;
    float yaw;
    float pitch;
    float latency_time;
    uint16_t checksum;
} vision_receive_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t reserved;
    float yaw;
    float pitch;
    float roll;
    uint16_t checksum;
} vision_transmit_packet_t;


typedef struct
{
    vision_receive_packet_t receive_packet;
    vision_transmit_packet_t transmit_packet;
}vision_data_t;

extern vision_data_t visionData;

void vision_init(void);
void Vision_GetRxPacket(const uint8_t *rx_data,vision_receive_packet_t *packet);
void Vision_SendData(vision_data_t *send,float yaw,float pitch,float roll);
#endif //RADAR_GIMBAL_H7_VISION_TASK_H