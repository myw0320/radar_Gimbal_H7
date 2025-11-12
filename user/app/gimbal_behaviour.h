//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#define RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H

#include "gimbal_task.h"
typedef enum
{
    GIMBAL_INIT_MODE,//云台初始化
    GIMBAL_MANUAL_MODE,//手动控制
    GIMBAL_AUTO_MODE,//自动模式
    GIMBAL_INABILITY_MODE,//云台无力
}gimbal_mode_enum;

extern gimbal_mode_enum gimbalMode;//云台模式
void gimbal_mode_update(gimbal_control_struct *mode);
#endif //RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H