/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_OUT2_EN_Pin GPIO_PIN_13
#define Power_OUT2_EN_GPIO_Port GPIOC
#define Power_OUT1_EN_Pin GPIO_PIN_14
#define Power_OUT1_EN_GPIO_Port GPIOC
#define Power_5V_EN_Pin GPIO_PIN_15
#define Power_5V_EN_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_0
#define CLK_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOA
#define DO_Pin GPIO_PIN_9
#define DO_GPIO_Port GPIOE
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define ACC_INT_EXTI_IRQn EXTI15_10_IRQn
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define DI_Pin GPIO_PIN_13
#define DI_GPIO_Port GPIOE
#define LCD_LIGHT_Pin GPIO_PIN_14
#define LCD_LIGHT_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define Power_OUT1_ON 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define Power_OUT1_OFF 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

#define Power_OUT2_ON 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define Power_OUT2_OFF 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)

#define Power_5V_ON 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define Power_5V_OFF 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
