/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float min_out;
    float max_iout; //���������
    float min_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

    // 高级PID
    uint8_t IntegralSeparationEn;   // 积分分离使能
    float IntegralSeparationErr;    // 积分分离/变速积分误差阈值
    uint8_t VariableIntegralEn;     // 变速积分使能（优先级高于积分分离）

    // 抗低频震荡
    uint8_t LeakyIntegralEn;        // 积分遗忘因子使能
    float Ki_decay;                 // 积分衰减系数 (0.95~0.99, 1.0=不衰减)
    uint8_t TrapezoidalIntegralEn;  // 梯形积分使能

}pid_struct;

void PID_init(pid_struct *pid, uint8_t mode, const float PID[3], float max_out, float min_out, float max_iout, float min_iout);
void PID_advanced_config(pid_struct *pid, uint8_t integral_sep_en, float integral_sep_err, uint8_t variable_integral_en);
void PID_integral_config(pid_struct *pid, uint8_t leaky_en, float ki_decay, uint8_t trapezoidal_en);
float PID_calc(pid_struct *pid, float ref, float set);
void PID_clear(pid_struct *pid);
#endif
