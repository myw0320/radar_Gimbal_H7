//
// Created by myw04 on 2025/11/7.
//

#include "buzzer.h"
#include "tim.h"
/*******蜂鸣器*********/
buzzer_struct buzzerMessage;

/**开机音效序列**/
sound_struct startSound[16] =
{
    { 466, 100 },
    { 587, 150 },

    { 740, 150 },
    { 880, 150 },
    { 1175, 200 },

    { 0, 100 },
    { 1175, 240 },
    { 0, 100 }
};
/**过压警报音效序列**/
sound_struct overVoltSound[12] =
{
    { 880, 150 },
    { 0, 50 },
    { 880, 150 },
    { 0, 50 },
    { 880, 150 },
    { 0, 500 },
};


sound_struct underVoltSound[12] =
{
    { 880, 150 },
    { 0, 50 },
    { 880, 150 },
    { 0, 50 },
    { 880, 150 },
    { 0, 500 },
};
void Buzzer_FreqSet(uint16_t freq)
{
    // uint32_t timer_clock = 170000000;//主时钟
    // uint32_t pwm_period = timer_clock / freq / (htim1.Init.Prescaler+1) - 1;
    // if (freq == 0)
    // {
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
    // }
    // else
    // {
    //     __HAL_TIM_SET_AUTORELOAD(&htim1,pwm_period);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_period / 2);
    // }
}

//音调更新
void Buzzer_FreqUpdate(buzzer_struct *sound,buzzer_mode_enum mode,sound_struct *type,uint8_t len)
{
    switch (sound->buzzerStatus)
    {
        case BUZZER_IDLE:
        {
            sound->index = 0;
            sound->currentTime = 0;
            sound->buzzerStatus = BUZZER_PLAY;
            break;
        }
        case BUZZER_PLAY:
        {
            Buzzer_FreqSet(type[sound->index].freq);
            sound->currentTime ++;
            if (sound->currentTime >= type[sound->index].duration)
            {
                // 切换到下一个音符
                sound->index++;
                sound->currentTime = 0;
            }
            // 检查是否到达音效末尾
            if (sound->index >= len)
            {
                if (mode == CIRCLES || sound->index != len)
                {
                    //循环模式重回空闲状态
                    sound->buzzerStatus = BUZZER_IDLE;
                }
                else
                {
                    sound->buzzerStatus = BUZZER_STOP;
                }
            }
            break;
        }
        case BUZZER_STOP:
        {
            if (sound->index != len)
            {
                sound->buzzerStatus = BUZZER_IDLE;
            }
            Buzzer_FreqSet(0);
            break;
        }
    }
}
