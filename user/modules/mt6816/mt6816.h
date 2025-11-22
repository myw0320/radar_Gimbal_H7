#ifndef RADAR_GIMBAL_H7_MT6816_H_
#define RADAR_GIMBAL_H7_MT6816_H_

#include "main.h"
#include "stdbool.h"
#include "spi.h"
#include "gpio.h"

// #define MT6816_SPI_CS_H()	     HAL_GPIO_WritePin(MT6816_NSS_GPIO_Port,MT6816_NSS_Pin,GPIO_PIN_SET)
// #define MT6816_SPI_CS_L()		 HAL_GPIO_WritePin(MT6816_NSS_GPIO_Port,MT6816_NSS_Pin,GPIO_PIN_RESET)

#define MT6816_Mode_SPI		     (0x03)


typedef struct
{
	uint16_t	sample_data;	
	uint16_t	angle;				
	bool		no_mag_flag;	
	bool		pc_flag;			
}mt6816_signal_t;

void REIN_MT6816_SPI_Signal_Init(void);		
void RINE_MT6816_SPI_Get_AngleData(void);	

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *GPIO_Port;//¶¨ÒåGPIOÒý½Å
	uint16_t GPIO_Pin;
	mt6816_signal_t Signal;
	uint16_t	angle_data;
}mt6816_data_struct;

extern mt6816_data_struct mt6816Data;




#endif //RADAR_GIMBAL_H7_MT6816_H_


