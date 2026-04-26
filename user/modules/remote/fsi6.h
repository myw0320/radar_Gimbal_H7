#ifndef RADAR_GIMBAL_H7_REMOTE_H
#define RADAR_GIMBAL_H7_REMOTE_H
#include "bsp_uart.h"


#define SBUS_FS_RX_LEN 50U
#define FSI6_RX_LEN 25u


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


/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 */
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
    struct __attribute__((packed))
    {
        int16_t ch[4];
        uint8_t sw[4];
        uint16_t rotory_sw[2];
    }rc;
    uint16_t crc16;
    struct __attribute__((packed))
    {
        uint16_t CH[16];
    } SBUS;
}fsi6_data_struct;


extern fsi6_data_struct fsi6Data;
extern uint8_t fsi6_rx_buf[2][SBUS_FS_RX_LEN];//控数据包
void Fsi6_Init(void);
void Fsi6_RxPacketUpdate(volatile const uint8_t *sbus_buf, fsi6_data_struct *rc_ctrl);
void Remote_IRQHandler(UART_HandleTypeDef *huart,uint16_t Size);
#endif //RADAR_GIMBAL_H7_REMOTE_H