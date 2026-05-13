//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#define RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#include "main.h"
#include "gimbal_task.h"
#include "controller.h"
#include "user_lib.h"
#define YAW_RC_SEN  0.000001f//提高雷达角度分辨率
#define PITCH_RC_SEN  0.000001f

#define YAW_MOUSE_SEN  0.00005f
#define PITCH_MOUSE_SEN  0.00005f
//时间ms转s
#define TIME_MS_TO_S(ms) (float)(ms / 1000.0f)

typedef enum
{
    GIMBAL_STOP = 0,
    GIMBAL_INIT,
    GIMBAL_AUTO,
    GIMBAL_MANUAL,//手动控制
}gimbal_control_mode_enum;
typedef enum
{
    GIMBAL_NO_MOVE_MODE = 0,//云台无力
    GIMBAL_INIT_MODE,//云台初始化
    GIMBAL_MANUAL_RC_MODE,//云台手动控制
    GIMBAL_AUTO_MOVE_MODE,//自动移动
    GIMBAL_AUTO_SCAN_MODE,//自动扫描
    GIMBAL_AUTO_ATTACK_MODE = 5,//自动攻击
}gimbal_mode_enum;

 extern gimbal_mode_enum gimbalMode;//云台模式
void gimbal_motor_mode_update(gimbal_control_struct *motor_mode_update);
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set);
#endif //RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H