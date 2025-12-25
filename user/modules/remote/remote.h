#ifndef RADAR_GIMBAL_H7_REMOTE_H
#define RADAR_GIMBAL_H7_REMOTE_H
#include "main.h"
#include "bsp_uart.h"


#define SBUS_RX_LEN 50U
#define REMOTE_RX_LEN 25u


#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)


#define rc_deadband_limit(input, output, dealine)        \
{                                                    \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
    (output) = (input);                          \
}                                                \
else                                             \
{                                                \
(output) = 0;                                \
}                                                \
}

typedef struct __attribute__((packed))
{
    __attribute__((packed)) struct
    {
        int16_t ch[4];
        uint8_t sw[4];
        uint16_t rotory_sw[2];
        // uint8_t mode_sw;
        // uint8_t stop;
        // uint8_t left_button;
        // uint8_t	right_button;
        // int16_t wheel;
        // uint8_t shutter;
    }rc;
    __attribute__((packed)) struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t middle;
    } mouse;
    __attribute__((packed)) struct
    {
        uint16_t v;
    } key;
    uint16_t crc16;
    __attribute__((packed)) struct
    {
        uint16_t CH[16];
    } SBUS;
}rc_ctrl_struct;


extern rc_ctrl_struct rcData;
extern uint8_t remote_rx_buf[2][SBUS_RX_LEN];//¿ØÊı¾İ°ü
void Remote_Init();
void Remote_RxPacketUpdate(volatile const uint8_t *sbus_buf, rc_ctrl_struct *rc_ctrl);
void Remote_IRQHandler(UART_HandleTypeDef *huart,uint16_t Size);
#endif //RADAR_GIMBAL_H7_REMOTE_H