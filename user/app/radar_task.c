
#include "radar_task.h"

#define RADAR_RX_LEN 28u
uint8_t radar_rx_buf[2][RADAR_RX_LEN];//雷达数据包
radar_data_struct radarData;

void radar_task(void *pvParameters)
{

}

void radar_init(void)
{

}

void Radar_RxPacketUpdate(const uint8_t *rx_data,radar_data_struct *update)
{
    if (rx_data[0] == 0xBB)
    {

    }
}
//雷达数据控制云台
void radar_to_gimbal(float *yaw,float *pitch,radar_data_struct *gimbal_control)
{

}