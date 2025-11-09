//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_H7_GIMBAL_TASK_H
#define RADAR_GIMBAL_H7_GIMBAL_TASK_H
#include "stdbool.h"
#include "vision.h"
#include "DM_motor.h"
typedef enum
{
    GIMBAL_INIT_MODE,//云台初始化
    GIMBAL_MANUAL_MODE,//手动控制
    GIMBAL_AUTO_MODE,//自动模式
    GIMBAL_INABILITY_MODE,//云台无力
}gimbal_mode_enum;

//单个欧拉角结构体
typedef struct
{
    float absolute_angle;
    float absolute_angle_set;
    float absolute_angle_max;
    float absolute_angle_min;

    float relative_angle;
    float relative_angle_set;
    float relative_angle_max;
    float relative_angle_min;
}euler_angle_struct;

typedef struct
{
    //vision_data_struct *visionData;//视觉数据
    euler_angle_struct yawEulerAngle;
    dm_control_struct yawMotor;//yaw电机结构体
    euler_angle_struct pitchEulerAngle;
    dm_control_struct pitchMotor;//pitch电机结构体

    bool enable;
}gimbal_control_struct;

void gimbal_task(void);
#endif //RADAR_GIMBAL_H7_GIMBAL_H