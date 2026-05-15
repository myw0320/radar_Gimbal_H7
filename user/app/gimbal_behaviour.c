//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_behaviour.h"

#include "detect_task.h"
#include "fsi6.h"
#include <math.h>

#define RADAR_REACHED_YAW_ERR 0.08f
#define RADAR_REACHED_PITCH_ERR 0.06f
#define AUTO_SCAN_YAW_RANGE 0.35f
#define AUTO_SCAN_PITCH_RANGE 0.20f
#define AUTO_SCAN_YAW_PERIOD 4.0f
#define AUTO_SCAN_PITCH_STEP 0.04f

/* 全局变量 --------------------------------------------------------------- */
gimbal_control_mode_enum gimbalControlMode = GIMBAL_STOP;       // 云台控制模式
gimbal_control_mode_enum gimbalControlModeLast = GIMBAL_STOP;   // 上一次云台控制模式

gimbal_mode_enum gimbalMode = GIMBAL_INIT_MODE;                 // 云台当前模式

/* 静态函数声明 ------------------------------------------------------------- */
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_manual_rc_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_stop_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_scan_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_scan_reset(gimbal_control_struct *control);
static uint8_t gimbal_radar_reached(const gimbal_control_struct *control);

/**
 * @brief  根据遥控器开关设置云台行为模式
 * @param  behaver 云台控制结构体指针
 */
void gimbal_behaviour_set(gimbal_control_struct *behaver)
{
    static gimbal_mode_enum last_mode = GIMBAL_INIT_MODE;

    if (behaver == NULL)
    {
        return;
    }
    last_mode = gimbalMode;
    // 读取遥控器两位开关，组合为模式选择位
    uint8_t mode_bit = ((behaver->rc_fsi6_point->rc.sw[0]<<1) | behaver->rc_fsi6_point->rc.sw[1]);
    gimbalControlModeLast = gimbalControlMode;  // 记录上一次模式

    // 开关组合 → 控制模式
    switch (mode_bit)
    {
        case 0:
            gimbalControlMode = GIMBAL_STOP;
            break;
        case 2:
            gimbalControlMode = GIMBAL_STOP;
            break;
        case 1:
            gimbalControlMode = GIMBAL_AUTO;
            break;
        case 3:
            gimbalControlMode = GIMBAL_MANUAL;
            break;
    }

    // 控制模式 → 云台子模式
    switch(gimbalControlMode)
    {
        case GIMBAL_STOP:
        {
            gimbalMode = GIMBAL_NO_MOVE_MODE;
            break;
        }
        case GIMBAL_INIT:  // 初始化
        {
            gimbalMode = GIMBAL_INIT_MODE;
            break;
        }
        case GIMBAL_MANUAL:  // 手动模式细分
        {
            switch (behaver->rc_fsi6_point->rc.sw[2])
            {
                case RC_SW_UP:
                {
                    gimbalMode = GIMBAL_MANUAL_MOVE_MODE;
                    break;
                }
                case RC_SW_MID:
                {
                    gimbalMode = GIMBAL_MANUAL_ATTACK_MODE;
                    break;
                }
                case RC_SW_DOWN:
                {

                    break;
                }
            }
            break;
        }
        case GIMBAL_AUTO:  // 自动模式细分
        {
            if (!toe_is_error(VISION_TOE) &&
                behaver->vision_point->receive_packet.packet_state == DEC_OK)
            {
                gimbalMode = GIMBAL_AUTO_ATTACK_MODE;
            }
            else if (!toe_is_error(RADAR_TOE) &&
                     behaver->radar_point->receive_packet.packet_state == DEC_OK)
            {
                gimbalMode = gimbal_radar_reached(behaver) ? GIMBAL_AUTO_SCAN_MODE : GIMBAL_AUTO_MOVE_MODE;
            }
            else
            {
                gimbalMode = GIMBAL_AUTO_SCAN_MODE;
            }
            break;

            // 雷达数据有效
            if (!toe_is_error(RADAR_TOE) && behaver->radar_point->receive_packet.packet_state == DEC_OK)
            {
                if (!toe_is_error(VISION_TOE))  // 视觉数据有效
                {
                    if (behaver->radar_point->receive_packet.packet_state == DEC_OK)  // 识别到目标
                    {
                        gimbalMode = GIMBAL_AUTO_ATTACK_MODE;
                    }
                    else if (behaver->radar_point->receive_packet.packet_state == DEC_DATA_NO)  // 未识别目标
                    {
                        gimbalMode = GIMBAL_AUTO_SCAN_MODE;
                    }
                }
                else
                {
                    gimbalMode = GIMBAL_AUTO_MOVE_MODE;
                }

            }

        }
    }

    if (last_mode != GIMBAL_AUTO_SCAN_MODE && gimbalMode == GIMBAL_AUTO_SCAN_MODE)
    {
        gimbal_scan_reset(behaver);
    }
}

/**
 * @brief  更新云台电机工作模式
 * @param  motor_mode_update 云台控制结构体指针
 */
void gimbal_motor_mode_update(gimbal_control_struct *motor_mode_update)
{
    if (motor_mode_update == NULL)
    {
        return;
    }
    // 更新云台行为模式
    gimbal_behaviour_set(motor_mode_update);

    // 根据云台模式设定电机模式
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_INIT;
            motor_mode_update->pitchEuler.motorMode = MOTOR_INIT;
            break;
        }
        case GIMBAL_MANUAL_ATTACK_MODE:
        case GIMBAL_MANUAL_MOVE_MODE:
        case GIMBAL_AUTO_MOVE_MODE:
        case GIMBAL_AUTO_SCAN_MODE:
        case GIMBAL_AUTO_ATTACK_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_GYRO;
            motor_mode_update->pitchEuler.motorMode = MOTOR_GYRO;
            break;
        }
    }

    // 记录上一次电机模式
    motor_mode_update->yawEuler.last_motorMode = motor_mode_update->yawEuler.motorMode;
    motor_mode_update->pitchEuler.last_motorMode = motor_mode_update->pitchEuler.motorMode;
}

/**
 * @brief  根据当前云台模式执行对应的控制策略
 * @param  add_yaw          偏航角增量输出
 * @param  add_pitch        俯仰角增量输出
 * @param  gimbal_control_set 云台控制结构体指针
 */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    switch(gimbalMode)
    {
        case GIMBAL_NO_MOVE_MODE:  // 静止
        {
            gimbal_stop_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_OFF;  // 关闭激光
            break;
        }
        case GIMBAL_INIT_MODE:  // 初始化
        {
            gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_OFF;  // 关闭激光
            break;
        }
        case GIMBAL_MANUAL_MOVE_MODE:  // 手动移动
        {
            gimbal_manual_rc_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_ON;  // 关闭激光
            break;
        }
        case GIMBAL_AUTO_MOVE_MODE:  // 自动移动
        {
            gimbal_auto_move_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_OFF;  // 关闭激光
            break;
        }
        case GIMBAL_AUTO_SCAN_MODE:  // 自动扫描
        {
            gimbal_auto_scan_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_OFF;  // 关闭激光
            break;
        }
        case GIMBAL_MANUAL_ATTACK_MODE:  // 手动攻击
        case GIMBAL_AUTO_ATTACK_MODE:    // 自动攻击
        {
            gimbal_auto_attack_control(add_yaw, add_pitch, gimbal_control_set);
            Power_5V_ON;  // 打开激光
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * @brief  云台初始化控制 — 回绝对零位
 */
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    *yaw = control->yawEuler.absolute_zero_angle;
    *pitch = control->pitchEuler.absolute_zero_angle;
}

/**
 * @brief  云台停止控制 — 角度增量置零
 */
static void gimbal_stop_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief  手动遥控控制 — 将遥控器摇杆值映射为角度增量
 */
static void gimbal_manual_rc_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    // 遥控器死区限幅
    rc_deadband_limit(control->rc_fsi6_point->rc.ch[0], yaw_channel, 20);
    rc_deadband_limit(control->rc_fsi6_point->rc.ch[1], pitch_channel, 20);

    // 通道值 → 角度增量（带衰减系数）
    *yaw = (float)-yaw_channel * YAW_RC_SEN;
    *pitch = (float)-pitch_channel * PITCH_RC_SEN;
}

/**
 * @brief  自动移动控制 — 直接跟随雷达目标角度
 */
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    *yaw = control->radar_point->receive_packet.yaw;
    *pitch = control->radar_point->receive_packet.pitch;
}

/**
 * @brief  扫描轨迹计算 — 锯齿波扫描 + 逐行步进
 * @param  scan_control 扫描控制结构体
 * @param  range_yaw   偏航扫描范围
 * @param  range_pitch 俯仰扫描范围
 * @param  period_yaw  偏航扫描周期
 * @param  run_time    已运行时间
 * @note   偏航轴做三角波往复运动，每次换向时俯仰轴步进一行
 */
void scan_control_set(scan_struct *scan_control, float range_yaw, float range_pitch, float period_yaw, float run_time)
{
    // 1. 计算偏航轴当前目标位置（三角波轨迹）
    float step_yaw = 4.0f * range_yaw / period_yaw;
    float calc_time = run_time - period_yaw * ((int16_t)(run_time / period_yaw));

    float next_yaw = 0;
    int8_t current_dir = 1;  // 当前运动方向：1=正向，-1=反向

    if (calc_time < 0.25f * period_yaw) {
        next_yaw = step_yaw * calc_time;
        current_dir = 1;
    }
    else if (calc_time < 0.75f * period_yaw) {
        next_yaw = -step_yaw * calc_time + 2.0f * range_yaw;
        current_dir = -1;
    }
    else {
        next_yaw = step_yaw * calc_time - 4.0f * range_yaw;
        current_dir = 1;
    }

    // 2. 检测方向翻转，每次换向俯仰步进一个增量
    if (current_dir != scan_control->last_yaw_dir)
    {
        scan_control->pitch_accumulated += AUTO_SCAN_PITCH_STEP;

        // 俯仰超限则回绕
        if (scan_control->pitch_accumulated > range_pitch)
        {
            scan_control->pitch_accumulated = -range_pitch;
        }
    }

    // 3. 记录当前方向供下次比较
    scan_control->last_yaw_dir = current_dir;

    // 4. 输出设定值
    scan_control->auto_scan_AC_set_yaw = next_yaw;
    scan_control->auto_scan_AC_set_pitch = scan_control->pitch_accumulated;
}

/**
 * @brief  自动扫描控制 — 输出扫描角度增量
 */
static void gimbal_auto_scan_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }

    // 更新扫描运行时间
    control->gimbalScan.scan_run_time = HAL_GetTick() * 0.001f - control->gimbalScan.scan_begin_time;

    // 计算当前扫描位置
    scan_control_set(&control->gimbalScan, AUTO_SCAN_YAW_RANGE, AUTO_SCAN_PITCH_RANGE,
                     AUTO_SCAN_YAW_PERIOD, control->gimbalScan.scan_run_time);

    *yaw = control->gimbalScan.yaw_center_value + control->gimbalScan.auto_scan_AC_set_yaw;
    *pitch = control->gimbalScan.pitch_center_value + control->gimbalScan.auto_scan_AC_set_pitch;
}

/**
 * @brief  自动攻击控制 — 跟随视觉目标角度
 * @note   当视觉数据为无效值(≥6.28)时保持上一次有效值
 */
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }

    *yaw = control->vision_point->receive_packet.yaw;
    *pitch = -control->vision_point->receive_packet.pitch;
}

static void gimbal_scan_reset(gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return;
    }

    if (!toe_is_error(RADAR_TOE) && control->radar_point->receive_packet.packet_state == DEC_OK)
    {
        control->gimbalScan.yaw_center_value = control->radar_point->receive_packet.yaw;
        control->gimbalScan.pitch_center_value = control->radar_point->receive_packet.pitch;
    }
    else
    {
        control->gimbalScan.yaw_center_value = control->yawEuler.absolute_angle;
        control->gimbalScan.pitch_center_value = control->pitchEuler.absolute_angle;
    }

    control->gimbalScan.scan_begin_time = HAL_GetTick() * 0.001f;
    control->gimbalScan.scan_run_time = 0.0f;
    control->gimbalScan.auto_scan_AC_set_yaw = 0.0f;
    control->gimbalScan.auto_scan_AC_set_pitch = 0.0f;
    control->gimbalScan.pitch_accumulated = -AUTO_SCAN_PITCH_RANGE;
    control->gimbalScan.last_yaw_dir = 1;
}

static uint8_t gimbal_radar_reached(const gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return 0;
    }

    return fabsf(control->radar_point->receive_packet.yaw - control->yawEuler.absolute_angle) < RADAR_REACHED_YAW_ERR &&
           fabsf(control->radar_point->receive_packet.pitch - control->pitchEuler.absolute_angle) < RADAR_REACHED_PITCH_ERR;
}
