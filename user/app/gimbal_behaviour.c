//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_behaviour.h"
#include "fsi6.h"
gimbal_control_mode_enum gimbalControlMode = GIMBAL_STOP;//云台控制模式
gimbal_control_mode_enum gimbalControlModeLast = GIMBAL_STOP;

gimbal_mode_enum gimbalMode = GIMBAL_INIT_MODE;//云台模式


static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_absolute_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_stop_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_scan_control(float *yaw, float *pitch, gimbal_control_struct *control);
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control);


void gimbal_behaviour_set(gimbal_control_struct *behaver)
{
    if (behaver == NULL)
    {
        return;
    }
    //uint8_t mode_bit = ((behaver->rc_point->rc.sw[0]<<1) | behaver->rc_point->rc.sw[1]);
    uint8_t mode_bit = behaver->rc_point->rc.sw[0];
    static gimbal_mode_enum currentMode = GIMBAL_AUTO_SCAN_MODE;//初始化为扫描
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
    gimbalControlModeLast = gimbalControlMode;//保存上次数据

    switch(gimbalControlMode)
    {
        case GIMBAL_STOP:
        {
            gimbalMode = GIMBAL_NO_MOVE_MODE;
            break;
        }
        case GIMBAL_INIT://自动移动
        {
            gimbalMode = GIMBAL_INIT_MODE;
            break;
        }
        case GIMBAL_MANUAL:
        {
            gimbalMode = GIMBAL_MANUAL_RC_MODE;
            break;
        }
        case GIMBAL_AUTO://自动移动
        {
            if (behaver->rc_point->rc.sw[1] == 1)
            {
                if (!behaver->vision_point->receive_packet.x && !behaver->vision_point->receive_packet.y && !behaver->vision_point->receive_packet.ok_flag)
                {
                    gimbalMode = GIMBAL_AUTO_SCAN_MODE;
                }
                else
                {
                    gimbalMode = GIMBAL_AUTO_ATTACK_MODE;
                }
            }
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
    if (motor_mode_update == NULL)
    {
        return;
    }
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
        case GIMBAL_MANUAL_RC_MODE:
        case GIMBAL_AUTO_SCAN_MODE:
        case GIMBAL_AUTO_ATTACK_MODE:
        case GIMBAL_AUTO_MOVE_MODE:
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
    }
    //保存
    motor_mode_update->yawEuler.last_motorMode = motor_mode_update->yawEuler.motorMode;
    motor_mode_update->pitchEuler.last_motorMode = motor_mode_update->pitchEuler.motorMode;
}

void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    switch(gimbalMode)
    {
        case GIMBAL_NO_MOVE_MODE://无力
        {
            gimbal_stop_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_INIT_MODE:
        {
            gimbal_init_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_MANUAL_RC_MODE:
        {
            gimbal_absolute_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_MOVE_MODE://自动移动
        {
            gimbal_auto_move_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_SCAN_MODE://自动扫描
        {
            gimbal_auto_scan_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_ATTACK_MODE://自动攻击
        {
            gimbal_auto_attack_control(add_yaw,add_pitch,gimbal_control_set);
            break;
        }
        default:
        {
            break;
        }
    }
}

//云台初始化
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    //初始化状态控制量计算
    *yaw = control->yawEuler.absolute_zero_angle;
    *pitch = control->pitchEuler.absolute_zero_angle;
}
static void gimbal_stop_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    //云台初始化校准
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
//手动控制
static void gimbal_absolute_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(control->rc_point->rc.ch[0],yaw_channel,10);
    rc_deadband_limit(control->rc_point->rc.ch[1],pitch_channel,10);
    //遥控器数据处理
    *yaw = (float)-yaw_channel* YAW_RC_SEN;//衰减系数
    *pitch = (float)-pitch_channel * PITCH_RC_SEN;

}
static void gimbal_relative_control(float *yaw, float *pitch, gimbal_control_struct *control)
{

}
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
//自动移动模式
static void gimbal_auto_move_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    //雷达控制云台
    PID_calc(&control->radar_point->yaw_err_pid,control->radar_point->receive_packet.yaw,0);
    PID_calc(&control->radar_point->pitch_err_pid,control->radar_point->receive_packet.pitch,0);
    *yaw = control->radar_point->yaw_err_pid.out;
    *pitch = control->radar_point->pitch_err_pid.out;
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
    float pitch_error = 0;
    float yaw_error = 0;

    // pitch轴yaw轴设定角度
    float pitch_set_angle = 0;
    float yaw_set_angle = 0;


    // 计算过去设定角度与当前角度之间的差值
    yaw_error = control->yawEuler.absolute_angle_set - control->yawEuler.absolute_angle;
    pitch_error = control->pitchEuler.absolute_angle_set - control->pitchEuler.absolute_angle;

    // 自动扫描设置浮动值
    float auto_scan_AC_set_yaw = 0;
    float auto_scan_AC_set_pitch = 0;
    // 计算运行时间
    control->gimbalScan.scan_run_time = HAL_GetTick()*0.001f - control->gimbalScan.scan_begin_time;

    // 云台自动扫描,设置浮动值
    scan_control_set(&auto_scan_AC_set_yaw, 1.4f, 6, control->gimbalScan.scan_run_time);
    scan_control_set(&auto_scan_AC_set_pitch,0.2f, 4, control->gimbalScan.scan_run_time);
    // 赋值控制值  = 中心值 + 加上浮动函数
    yaw_set_angle = auto_scan_AC_set_yaw;
    pitch_set_angle = auto_scan_AC_set_pitch;
    // 赋值增量
    *yaw = yaw_set_angle - control->yawEuler.absolute_angle- yaw_error;
    *pitch = pitch_set_angle - control->pitchEuler.absolute_angle - pitch_error;
}



//自动攻击模式
static void gimbal_auto_attack_control(float *yaw, float *pitch, gimbal_control_struct *control)
{
    if (yaw == NULL || pitch == NULL || control == NULL)
    {
        return;
    }
    //获取时间
    control->visionToGimbal.current_time = HAL_GetTick()/1000.0f;
    // current_x = Math_Constrain(&current_x,-800,640);
    // current_y = Math_Constrain(&current_y,-804,276);
    // //视觉控制云台
    if (fabs(control->visionToGimbal.now_x - control->visionToGimbal.target_x) < 10 &&
        fabs(control->visionToGimbal.now_y - control->visionToGimbal.target_x) <10 &&
        fabs(control->visionToGimbal.current_time - control->vision_point->receive_packet.receive_time) <1.0f)//判断是否开启激光
    {
        Power_OUT1_ON;
        Power_OUT1_ON;
    }
    else
    {
        Power_OUT1_OFF;
        Power_OUT1_OFF;
    }

    //pid处理
    PID_calc(&control->visionToGimbal.x_err_pid,control->visionToGimbal.now_x,control->visionToGimbal.target_x);
    PID_calc(&control->visionToGimbal.y_err_pid,control->visionToGimbal.now_y,control->visionToGimbal.target_y);

    *yaw = control->visionToGimbal.x_err_pid.out / 180.0f * PI;
    *pitch = -control->visionToGimbal.y_err_pid.out / 180.0f * PI;
}