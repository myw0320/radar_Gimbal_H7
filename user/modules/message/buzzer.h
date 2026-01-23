#ifndef RADAR_GIMBAL_BUZZER_H
#define RADAR_GIMBAL_BUZZER_H
#include "stdint.h"



/****蜂鸣器****/
#define BUZZER_HTIM htim12
typedef enum
{
    BUZZER_IDLE= 0,//空闲
    BUZZER_PLAY,
    BUZZER_STOP
  }buzzer_status_enum;

typedef enum
{
    SINGLE = 0,
    CIRCLES = 1,
  }buzzer_mode_enum;

// 音效序列结构体
typedef struct
{
    uint16_t freq;    // 音符频率
    uint16_t duration;// 持续时间(ms)
}sound_struct;

typedef struct
{
    uint32_t currentTime;
    uint8_t index;
    buzzer_status_enum buzzerStatus;//状态机
    //buzzer_mode_enum buzzerMode;//蜂鸣器模式
}buzzer_struct;

extern sound_struct startSound[16];
extern sound_struct underVoltSound[8];
extern sound_struct overVoltSound[12];
extern sound_struct overCurrSound[12];
extern sound_struct shortCircuit[12];
extern sound_struct canLostSound[10];

void Buzzer_Init(void);
void Buzzer_FreqReset(buzzer_struct *reset);
void Buzzer_FreqUpdate(buzzer_struct *sound,buzzer_mode_enum mode,sound_struct *type,uint8_t len);

#endif //RADAR_GIMBAL_BUZZER_H