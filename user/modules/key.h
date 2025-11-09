#ifndef RADAR_GIMBAL_H7_KEY_H
#define RADAR_GIMBAL_H7_KEY_H
#include "main.h"

/****按键*****/
#define K1 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)

typedef enum
{
  KEY_IDLE,//空闲
  KEY_PLAY,//单次
  KEY_PLAYING,//长按
}key_status_enum;

typedef struct
{
  key_status_enum keyStatus;//按键状态机
  uint8_t keyNum;//计数器
  uint8_t key;
  uint32_t startTime;
  uint32_t currentTime;
}key_struct;

extern key_struct keyMessage;
void Key_StatusUpdate(key_struct *key);
#endif //WS2812_H

