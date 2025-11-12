#ifndef RADAR_GIMBAL_H7_BSP_USART_H
#define RADAR_GIMBAL_H7_BSP_USART_H
#include "usart.h"
#include "string.h"

#define VISION_RX_LEN  28u
#define RADAR_RX_LEN 28u
#define REFEREE_RX_LEN 512u
#define REMOTE_RX_LEN 48u

typedef struct __attribute__((packed))
{
	uint8_t header;
	float  vx;
	float  vy;
	float  ang_z;
    //uint8_t spin_flag;
} vision_rxfifo_t;



extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);




#endif
