#include "vision_task.h"

#include "usart.h"


uint8_t vision_rx_buf[2][VISION_RX_LEN];//视觉数据包

vision_data_struct visionData;


void vision_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)vision_rx_buf[0],(uint32_t *)vision_rx_buf[1],VISION_RX_LEN);
}

void Vision_RxPacketUpdate(const uint8_t *rx_data,vision_data_struct *update)
{
    if (rx_data[0] == 0xAA)//检查包头
    {
        update->x = (float)(rx_data[1]<<8 | rx_data[2])/10.0f;
        update->y = (float)(rx_data[3]<<8 | rx_data[4])/10.0f;
    }
}

void vision_to_gimbal(float *yaw,float *pitch,vision_data_struct *gimbal_control)
{

}

void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{

}