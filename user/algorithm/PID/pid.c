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

#include "pid.h"
#include <math.h>

#define LimitMax(input, min, max)   \
    {                               \
        if (input > max)            \
        {                           \
            input = max;            \
        }                           \
        else if (input < min)       \
        {                           \
            input = min;            \
        }                           \
    }

void PID_init(pid_struct *pid, uint8_t mode, const float PID[3], float max_out, float min_out, float max_iout, float min_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->min_out = min_out;
    pid->max_iout = max_iout;
    pid->min_iout = min_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->IntegralSeparationEn = 0;
    pid->IntegralSeparationErr = 0.0f;
    pid->VariableIntegralEn = 0;
    pid->LeakyIntegralEn = 0;
    pid->Ki_decay = 1.0f;
    pid->TrapezoidalIntegralEn = 0;
}

void PID_advanced_config(pid_struct *pid, uint8_t integral_sep_en, float integral_sep_err, uint8_t variable_integral_en)
{
    if (pid == NULL)
    {
        return;
    }
    pid->IntegralSeparationEn = integral_sep_en;
    pid->IntegralSeparationErr = integral_sep_err;
    pid->VariableIntegralEn = variable_integral_en;
}


float PID_calc(pid_struct *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    // 计算积分项系数: 变速积分 > 积分分离 > 全积分
    float integral_coeff = 1.0f;
    if (pid->VariableIntegralEn)
    {
        if (fabsf(pid->error[0]) < pid->IntegralSeparationErr)
        {
            integral_coeff = (pid->IntegralSeparationErr - fabsf(pid->error[0])) / pid->IntegralSeparationErr;
        }
        else
        {
            integral_coeff = 0.0f;
        }
    }
    else if (pid->IntegralSeparationEn)
    {
        if (fabsf(pid->error[0]) > pid->IntegralSeparationErr)
        {
            integral_coeff = 0.0f;
        }
        else
        {
            integral_coeff = 1.0f;
        }
    }

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];

        if (fabs(pid->error[0]) < 0.0001f)
        {
            pid->Iout = 0;
        }
        else
        {
            pid->Iout += pid->Ki * pid->error[0] * integral_coeff;
        }
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        // 反算抗饱和: 输出超限时反向修正积分，避免积分卡死在边界
        if (pid->out > pid->max_out)
        {
            pid->Iout -= (pid->out - pid->max_out);
            pid->out = pid->max_out;
        }
        else if (pid->out < pid->min_out)
            {
            pid->Iout -= (pid->out - pid->min_out);
            pid->out = pid->min_out;
        }
        LimitMax(pid->Iout, pid->min_iout, pid->max_iout);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);

        if (fabs(pid->error[0]) < 0.0005f)
        {
            pid->Iout = 0;
        }
        else
        {
            pid->Iout = pid->Ki * pid->error[0] * integral_coeff;
        }
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        // 反算抗饱和: 输出超限时反向修正积分，避免积分卡死在边界
        if (pid->out > pid->max_out)
        {
            pid->Iout -= (pid->out - pid->max_out);
            pid->out = pid->max_out;
        }
        else if (pid->out < pid->min_out)
        {
            pid->Iout -= (pid->out - pid->min_out);
            pid->out = pid->min_out;
        }
    }
    return pid->out;
}

void PID_clear(pid_struct *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
