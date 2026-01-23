//
// Created by myw04 on 2025/11/7.
//

#include "buzzer.h"
#include "tim.h"
/*******蜂鸣器*********/


/**开机音效序列**/
sound_struct startSound[16] =
{
    { 466, 10 },
    { 587, 15 },

    { 740, 15 },
    { 880, 15 },
    { 1175, 20 },

    { 0, 10 },
    { 1175, 24 },
    { 0, 10 }
};
/**欠压警报音效序列**/
/**di..di...**/
sound_struct underVoltSound[8] =
{
    { 880, 40 },
    { 0, 20 },
    { 880, 40 },
    { 0, 30 },

};
/**过压警报音效序列**/
/**di..di..di......**/
sound_struct overVoltSound[12] =
{
    { 880, 15 },
    { 0, 5 },
    { 880, 15 },
    { 0, 5 },
    { 880, 15 },
    { 0, 50 },
};
/**过流警报音效序列**/
/**di---..di..di..**/
sound_struct overCurrSound[12] =
{
    { 880, 50 },
    { 0, 5 },
    { 880, 15 },
    { 0, 5 },
    { 880, 15 },
    { 0, 50 },
};
/**短路警报音效序列**/
sound_struct shortCircuit[12] =
{
    { 880, 15 },
    { 0, 5 },
    { 880, 15 },
    { 0, 5 },
    { 880, 50 },
    { 0, 30 },
};
/**CAN丢失警报音效序列**/
sound_struct canLostSound[10] =
{
    { 880, 15 },
    { 0, 5 },
    { 880, 55 },
    { 0, 40 },

};

void Buzzer_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim12);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void Buzzer_FreqSet(uint16_t freq)
{
    uint32_t timer_clock = 240000000;//主时钟
    uint32_t pwm_period = timer_clock / freq / (BUZZER_HTIM.Init.Prescaler+1) - 1;
    if (freq == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
    }
    else
    {
        __HAL_TIM_SET_AUTORELOAD(&htim12,pwm_period);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,pwm_period / 2);
    }
}

void Buzzer_FreqReset(buzzer_struct *reset)
{
    reset->buzzerStatus = BUZZER_IDLE;
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
            sound->currentTime ++;
            Buzzer_FreqSet(type[sound->index].freq);
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
