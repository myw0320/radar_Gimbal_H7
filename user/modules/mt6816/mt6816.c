#include "mt6816.h"


mt6816_data_t mt6816Data;

void mt6816_init(mt6816_data_struct *init,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	init->hspi = hspi;//初始化SPI
	init->GPIO_Port = GPIOx;//初始化引脚
	init->GPIO_Pin = GPIO_Pin;
	init->Signal.sample_data = 0;
	init->Signal.angle = 0;
}

void MT6816_GetRxPacket(mt6816_data_struct *sign)
{
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	for(uint8_t i=0; i<3; i++){
		//读取SPI数据

		sign->GPIO_Port->BSRR = sign->GPIO_Pin << ;//拉低CS

		HAL_SPI_TransmitReceive(sign->hspi, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
		MT6816_SPI_CS_H();
		MT6816_SPI_CS_L();
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
		MT6816_SPI_CS_H();
		sign->sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		//奇偶校验
		h_count = 0;
		for(uint8_t j=0; j<16; j++){
			if(sign->sample_data & (0x0001 << j))
				h_count++;
		}
		if(h_count & 0x01){
			sign->pc_flag = false;
		}
		else{
			sign->pc_flag = true;
			break;
		}
	}
	if(sign->pc_flag){
		sign->angle = sign->sample_data >> 2;
		sign->no_mag_flag = (bool)(sign->sample_data & (0x0001 << 1));
	}
}


float MT6816_RxPacetUpdate(void)
{
	RINE_MT6816_SPI_Get_AngleData();
}
