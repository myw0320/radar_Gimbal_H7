#ifndef RADAR_GIMBAL_H7_DT7_H
#define RADAR_GIMBAL_H7_DT7_H
#include "bsp_uart.h"

#define SBUS_DT_RX_LEN 36U
#define DT7_RX_LEN 18u

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[5];
        uint8_t sw[2];

    }rc;
    struct __attribute__((packed))
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t middle;
    } mouse;
    struct __attribute__((packed))
    {
        uint16_t v;
    } key;
    uint16_t crc16;
    struct __attribute__((packed))
    {
        uint16_t CH[16];
    } SBUS;
}dt7_data_struct;


extern uint8_t dt7_rx_buf[2][SBUS_DT_RX_LEN];//¿ØÊý¾Ý°ü
extern dt7_data_struct dt7Data;
void Dt7_Init(void);
#endif //RADAR_GIMBAL_H7_DT7_H