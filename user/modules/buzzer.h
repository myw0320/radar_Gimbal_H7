#ifndef RADAR_GIMBAL_BUZZER_H
#define RADAR_GIMBAL_BUZZER_H
#include "main.h"

/****蜂鸣器****/
// 音符频率定义 (单位: Hz)
#define NOTE_C5  523
#define NOTE_DS5 622
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_AS5 932

// 节拍时长定义 (单位: ms)
#define BEAT_250 1000
#define BEAT_200 100
#define BEAT_150 100

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
extern sound_struct overVoltSound[12];
extern sound_struct underVoltSound[12];



extern buzzer_struct buzzerMessage;
void Buzzer_Reset(buzzer_struct *reset);
void Buzzer_FreqUpdate(buzzer_struct *sound,buzzer_mode_enum mode,sound_struct *type,uint8_t len);

#endif //RADAR_GIMBAL_BUZZER_H