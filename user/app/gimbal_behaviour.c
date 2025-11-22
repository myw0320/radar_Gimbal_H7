//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_behaviour.h"

gimbal_mode_enum gimbalMode;//云台模式
gimbal_mode_enum last_gimbalMode;//云台模式


static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_rc_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_scan_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control);


void gimbal_behaviour_set(gimbal_control_struct *behaver)
{
    if (behaver == NULL)
    {
        return;
    }
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
            if (behaver->rc_point)
            {

            }
            break;
        }
        case GIMBAL_RC_MODE:
        {
            break;
        }
        case GIMBAL_AUTO_MOVE_MODE://自动移动
        {
            break;
        }
        case GIMBAL_AUTO_SCAN_MODE://自动扫描
        {
            break;
        }
        case GIMBAL_AUTO_ATTACK_MODE://自动攻击
        {
            break;
        }
        case GIMBAL_NO_MOVE_MODE://无力
        {
            break;
        }
        default:
        {
            break;
        }
    }
}


void gimbal_motor_mode_update(gimbal_control_struct *motor_mode_update)
{
    //更新云台行为
    gimbal_behaviour_set(motor_mode_update);
    //更新云台各电机行为
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_INIT;
            motor_mode_update->pitchEuler.motorMode = MOTOR_INIT;
            break;
        }
        case GIMBAL_AUTO_MOVE_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_GYRO;
            motor_mode_update->pitchEuler.motorMode = MOTOR_GYRO;
            break;
        }
        case GIMBAL_AUTO_SCAN_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_GYRO;
            motor_mode_update->pitchEuler.motorMode = MOTOR_GYRO;
            break;
        }
        case GIMBAL_AUTO_ATTACK_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_GYRO;
            motor_mode_update->pitchEuler.motorMode = MOTOR_GYRO;
            break;
        }
        case GIMBAL_NO_MOVE_MODE:
        {
            motor_mode_update->yawEuler.motorMode = MOTOR_GYRO;
            motor_mode_update->pitchEuler.motorMode = MOTOR_GYRO;
            break;
        }
        default:
        {
            break;
        }
    }
}

void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
            gimbal_init_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_RC_MODE:
        {
            break;
        }
        case GIMBAL_AUTO_MOVE_MODE://自动移动
        {
            gimbal_auto_move_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_SCAN_MODE://自动扫描
        {
            break;
        }
        case GIMBAL_AUTO_ATTACK_MODE://自动攻击
        {
            break;
        }
        case GIMBAL_NO_MOVE_MODE://无力
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

//
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    //云台初始化
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
}
//手动控制
#define YAW_RC_SEN  0.000005f
#define PITCH_RC_SEN  0.000005f

#define YAW_MOUSE_SEN  0.00005f
#define PITCH_MOUSE_SEN  0.00005f
static void gimbal_rc_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    static float rc_add_yaw, rc_add_pitch;
    static float mouse_add_yaw, mouse_add_pitch;
    //遥控器数据处理
    rc_add_yaw = (float)control->rc_point->rc.ch[3] * YAW_RC_SEN;//衰减系数
    rc_add_pitch = (float)control->rc_point->rc.ch[2] * PITCH_RC_SEN;
    //鼠标数据处理
    mouse_add_yaw = (float)control->rc_point->mouse.x * YAW_MOUSE_SEN;
    mouse_add_pitch = (float)control->rc_point->mouse.y * PITCH_MOUSE_SEN;

    *yaw = rc_add_yaw + mouse_add_yaw;
    *pitch = rc_add_pitch + mouse_add_pitch;
}

//自动移动模式
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    //雷达控制云台
    radar_to_gimbal(yaw,pitch,control->radar_point);
}
//自动扫描模式
void scan_control_set(float *gimbal_set, float range, float period, float run_time)
{
    // 计算单次运行的步长
    float step = 4.0f * range / period;

    // 判断云台设置浮动角度是否超过最大值,限制最大值
    if (*gimbal_set >= range)
    {
        *gimbal_set = range;
    }
    else if (*gimbal_set <= -range)
    {
        *gimbal_set = -range;
    }
    // 处理运行时间，将运行时间处理到一个周期内
    float calc_time = run_time - period * ((int16_t)(run_time / period));
    // 判断当前时间所处的位置，根据当前位置，判断数值计算方向
    if (calc_time < 0.25f * period)
    {
        *gimbal_set = step * calc_time;
    }
    else if (0.25f * period <= calc_time && calc_time < 0.75f * period)
    {
        *gimbal_set = -(step * calc_time) + 2 * range;
    }
    else if (0.75f * period <= calc_time)
    {
        *gimbal_set = step * calc_time - 4 * range;
    }
}

static void gimbal_auto_scan_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    // yaw pitch 轴设定值与当前值的差值
    float pitch_error = 0;
    float yaw_error = 0;

    // pitch轴yaw轴设定角度
    float pitch_set_angle = 0;
    float yaw_set_angle = 0;

    //设置扫描中值
    control->gimbalScan.pitch_center_value = 0.05f;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = control->yawEuler.absolute_angle_set - control->yawEuler.absolute_angle;
    pitch_error = control->pitchEuler.absolute_angle_set - control->pitchEuler.absolute_angle;
    // 自动扫描设置浮动值
    float auto_scan_AC_set_yaw = 0;
    float auto_scan_AC_set_pitch = 0;
    // 计算运行时间
    control->gimbalScan.scan_run_time = TIME_MS_TO_S(HAL_GetTick()) - control->gimbalScan.scan_begin_time;
    // 云台自动扫描,设置浮动值
    scan_control_set(&auto_scan_AC_set_yaw, control->gimbalScan.yaw_range, control->gimbalScan.scan_yaw_period, control->gimbalScan.scan_run_time);
    scan_control_set(&auto_scan_AC_set_pitch, control->gimbalScan.pitch_range, control->gimbalScan.scan_pitch_period, control->gimbalScan.scan_run_time);
    // 赋值控制值  = 中心值 + 加上浮动函数
    yaw_set_angle = auto_scan_AC_set_yaw + control->gimbalScan.yaw_center_value;
    pitch_set_angle = auto_scan_AC_set_pitch + control->gimbalScan.pitch_center_value;

    // 一阶低通使数据平滑
    //first_order_filter_cali(&control->gimbalScan.yaw_auto_scan_first_order_filter, yaw_set_angle);
    //first_order_filter_cali(&control->gimbalScan.pitch_auto_scan_first_order_filter, pitch_set_angle);

    // pitch_set_angle = gimbal_control_set->gimbal_auto_scan.pitch_center_value;
    // yaw_set_angle = gimbal_control_set->gimbal_auto_scan.yaw_center_value;

    // 赋值增量
    //*yaw = control->gimbalScan.yaw_auto_scan_first_order_filter.out - gimbal_control_set->YAW_.absolute_angle- yaw_error;
    //*pitch = control->gimbalScan.pitch_auto_scan_first_order_filter.out - gimbal_control_set->PITCH_.absolute_angle - pitch_error;
}

//自动攻击模式
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    //视觉控制云台
    vision_to_gimbal(yaw,pitch,control->vision_point);
}