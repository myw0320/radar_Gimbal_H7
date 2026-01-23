
#ifndef __H7_VT13_H
#define __H7_VT13_H
#include "bsp_uart.h"
#include "usart.h"

#define SBUS_VT_RX_LEN 42U
#define VT13_RX_LEN 21u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

typedef struct __attribute__((packed))
{
    __attribute__((packed)) struct
    {
        int16_t ch[4];
        uint8_t mode_sw;
        uint8_t stop;
        uint8_t left_button;
        uint8_t	right_button;
        int16_t wheel;
        uint8_t shutter;
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
    uint16_t crc16_check;

} vt13_data_stuct;

extern vt13_data_stuct vt13Data;

#endif //__H7_VT13_H