//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_H7_GIMBAL_TASK_H
#define RADAR_GIMBAL_H7_GIMBAL_TASK_H
#include "stdbool.h"
#include "INS_task.h"
#include "vision_task.h"
#include "DM_motor.h"
#include "gimbal_behaver.h"

#define ECD_RANGE 8191
#define HALF_ECD_RANGE 4096
#define MOTOR_ECD_TO_RAD 0.000766990394f //编码值转换为弧度值  2*PI/8192
//yaw电机零点编码值
#define YAW_ZERO_ENCODER 4096
//pitch电机零点编码值
#define PITCH_ZERO_ENCODER 4096
//yaw轴数据限幅值
#define YAW_ABS_MAX
#define YAW_ABS_MIN
#define YAW_REL_MAX
#define YAW_REL_MIN
//pitch轴数据限幅值
#define PITCH_ABS_MAX
#define PITCH_ABS_MIN
#define PITCH_REL_MAX
#define PITCH_REL_MIN

typedef enum
{
    MOTOR_INIT,
    MOTOR_GYRO,
    MOTOR_ENCODER,
}motor_mode_enum;
//单个欧拉角结构体
typedef struct
{
    motor_mode_enum motorMode;

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
    INS_t *imuData;//陀螺仪数据
    vision_data_struct *visionData;//视觉数据
    euler_angle_struct yawEulerAngle;
    dm_control_struct yawMotor;//yaw电机结构体
    euler_angle_struct pitchEulerAngle;
    dm_control_struct pitchMotor;//pitch电机结构体

    bool enable;
}gimbal_control_struct;

extern gimbal_control_struct gimbalControl;//云台控制
void gimbal_task(void);
#endif //RADAR_GIMBAL_H7_GIMBAL_H