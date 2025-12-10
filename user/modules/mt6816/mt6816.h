#ifndef RADAR_GIMBAL_H7_MT6816_H_
#define RADAR_GIMBAL_H7_MT6816_H_

#include "main.h"
#include "stdbool.h"
#include "spi.h"
#include "gpio.h"

#define MT6816_SPI  hspi3
#define MT6816_SPI_CS_H()	     HAL_GPIO_WritePin(SPI3_NSS1_GPIO_Port,SPI3_NSS1_Pin,GPIO_PIN_SET)
#define MT6816_SPI_CS_L()		 HAL_GPIO_WritePin(SPI3_NSS1_GPIO_Port,SPI3_NSS1_Pin,GPIO_PIN_RESET)
// 编码器分辨率 (14位: 2^14 = 16384)
#define ENCODER_RESOLUTION 16384.0f
// 定时器每 0.5 毫秒触发一次中断
#define TIMER_PERIOD_MS 0.5f


typedef struct
{
	uint16_t	sample_data;// 原始数据
	uint16_t	angle;// 14位角度值
	bool		no_mag_flag;// 磁场强度标志 (No Mag)
	bool		pc_flag;// 奇偶校验标志
	uint16_t    angle_value;
	// --- 新增字段用于速度计算 ---
	uint16_t prev_angle;  // 上一次的角度值
	int16_t speed_rpm;    // 计算出的速度 (RPM)
}mt6816_data_struct;


extern mt6816_data_struct mt6816Data;



void mt6816_init(mt6816_data_struct *init);
void MT6816_GetRxPacket(mt6816_data_struct *sign);
void MT6816_GetRpm(void);
#endif //RADAR_GIMBAL_H7_MT6816_H_


