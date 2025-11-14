#ifndef RADAR_GIMBAL_H7_BSP_PWM_H
#define RADAR_GIMBAL_H7_BSP_PWM_H
#include "stdint.h"
#include "tim.h"


void pwm_init(TIM_HandleTypeDef *htim);
void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#endif
