//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#define RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H

#include "gimbal_task.h"
#include "controller.h"
#include "user_lib.h"

//时间ms转s
#define TIME_MS_TO_S(ms) (float)(ms / 1000.0f)

typedef enum
{
    GIMBAL_STOP = 0,
    GIMBAL_INIT,
    GIMBAL_MANUAL,//手动控制
    GIMBAL_AUTO,
}gimbal_control_mode_enum;
typedef enum
{
    GIMBAL_NO_MOVE_MODE,//云台无力
    GIMBAL_INIT_MODE,//云台初始化
    GIMBAL_MANUAL_RC_MODE,//云台手动控制
    GIMBAL_AUTO_MOVE_MODE,//自动移动
    GIMBAL_AUTO_SCAN_MODE,//自动扫描
    GIMBAL_AUTO_ATTACK_MODE,//自动攻击
}gimbal_mode_enum;

// extern gimbal_mode_enum gimbalMode;//云台模式
void gimbal_motor_mode_update(gimbal_control_struct *motor_mode_update);
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set);
#endif //RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H