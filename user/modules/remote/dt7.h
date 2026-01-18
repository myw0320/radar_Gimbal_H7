#ifndef RADAR_GIMBAL_H7_DT7_H
#define RADAR_GIMBAL_H7_DT7_H
#include "usart.h"
#include "bsp_uart.h"

#define SBUS_DT_RX_LEN 50U
#define DT7_RX_LEN 25u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

typedef struct __attribute__((packed))
{
    __attribute__((packed)) struct
    {
        int16_t ch[5];
        uint8_t sw[2];
        //int16_t rotory_sw[2];
    }rc;
    uint16_t crc16;
    __attribute__((packed)) struct
    {
        uint16_t CH[16];
    } SBUS;
}dt7_data_struct;


extern uint8_t dt7_rx_buf[2][SBUS_DT_RX_LEN];//¿ØÊý¾Ý°ü
extern dt7_data_struct dt7Data;
void Dt7_Init(void);
#endif //RADAR_GIMBAL_H7_DT7_H