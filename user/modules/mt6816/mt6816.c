#include "mt6816.h"


mt6816_data_struct mt6816Data;

void mt6816_init(mt6816_data_struct *init)
{

}

void MT6816_GetRxPacket(mt6816_data_struct *encoder)
{
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	for(uint8_t i=0; i<3; i++)
	{
		//读取SPI数据
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&Mt6816_SPI, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, 200);
		MT6816_SPI_CS_H();

		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&Mt6816_SPI, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, 200);
		MT6816_SPI_CS_H();
		encoder->sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		//奇偶校验
		h_count = 0;
		for(uint8_t j=0; j<16; j++)
		{
			if(encoder->sample_data & (0x0001 << j))
				h_count++;
		}
		if(h_count & 0x01)
		{
			encoder->pc_flag = false;
		}
		else
		{
			encoder->pc_flag = true;
			break;
		}
	}
	if(encoder->pc_flag)
	{
		encoder->encoder_value = encoder->sample_data >> 2;
		encoder->no_mag_flag = (bool)(encoder->sample_data & (0x0001 << 1));
	}
}

//中断调用
void MT6816_GetRpm(void)
{
	// 1. 获取当前编码值
	MT6816_GetRxPacket(&mt6816Data);
	//计算弧度值（rad）
	mt6816Data.angle_rad = (float)mt6816Data.encoder_value * ENC_TO_RAD;
	//只有在数据通过奇偶校验时才进行计算
	if (mt6816Data.pc_flag)
	{
		uint16_t current_angle = mt6816Data.encoder_value;

		// 2. 计算角度差 (带符号)
		// 差值可能为正 (正转) 或负 (反转)
		int32_t angle_delta = (int32_t)current_angle - (int32_t)mt6816Data.prev_angle;

		// 3. 处理环绕 (Rollover)
		// 角度值是循环的 (0-16383)。如果转速快，可能会发生跨越 0 度的环绕。

		// 如果角度差 > 半圈 (8192)，说明是负向转动跨越了 0 (例如: 100 -> 16300)
		if (angle_delta > (ENCODER_RESOLUTION / 2))
		{
			// 修正为实际的负向差值: 16200 - 16384 = -184
			angle_delta -= ENCODER_RESOLUTION;
		}
		// 如果角度差 < -半圈 (-8192)，说明是正向转动跨越了 0 (例如: 16300 -> 100)
		else if (angle_delta < -(ENCODER_RESOLUTION / 2))
		{
			// 修正为实际的正向差值: -16200 + 16384 = 184
			angle_delta += ENCODER_RESOLUTION;
		}

		// 4. 计算角速度 (RPM)
		// RPM 公式:
		// 速度 (RPM) = ( 角度变化量 / 刻度分辨率 ) * ( 60 秒 / 时间间隔(秒) )
		// 时间间隔(秒) = TIMER_PERIOD_MS / 1000
		//
		// 简化计算: RPM = angle_delta * 60000 / (ENCODER_RESOLUTION * TIMER_PERIOD_MS)

		float rpm = (float)angle_delta * 60000.0f / (ENCODER_RESOLUTION * TIMER_PERIOD_MS);

		// 5. 存储结果并更新上一次的角度值
		mt6816Data.speed_rpm = (int16_t)rpm;
		mt6816Data.speed_omega = rpm * RPM_TO_OMEGA;//(rad/s)
		mt6816Data.prev_angle = current_angle;
	}
	// 如果奇偶校验失败，则跳过此次计算，保持上一次的速度值不变。
}