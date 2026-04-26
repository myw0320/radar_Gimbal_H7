

#ifndef H7_HIPNUC_H
#define H7_HIPNUC_H

#include "bsp_uart.h"
#include "hipnuc_dec.h"

#define HI04_UART huart8

#define UART8_RX_LEN 152u
#define HI04_RX_LEN 76u
typedef struct
{
    const hi91_t *hipnuc_date;
    uint8_t ok_flag;
    uint16_t uart_rx_index;
    float yaw;
    float pitch;
    float roll;

    float yaw_angle;
    float pitch_angle;

}hi04_imu_data_t;

void hi04_init(hi04_imu_data_t *init);
void Hi04_GetRxPacket(hi04_imu_data_t *packet);

#endif //H7_HIPNUC_H