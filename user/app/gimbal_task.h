//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_H7_GIMBAL_TASK_H
#define RADAR_GIMBAL_H7_GIMBAL_TASK_H
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "ins_task.h"
#include "vision_task.h"
// #include "radar_task.h"
#include "detect_task.h"
#include "remote.h"
#include "Dji_motor.h"
#include "DM_motor.h"
#include "pid.h"
#include "Mathh.h"
#include "user_lib.h"
#define ECD_RANGE 8191
#define HALF_ECD_RANGE 4096
#define MOTOR_ECD_TO_RAD 0.000766990394f //编码值转换为弧度值  2*PI/8192
//yaw电机零点编码值
#define YAW_ZERO_ENCODER 4096
//pitch电机零点编码值
#define PITCH_ZERO_ENCODER 4096
//yaw轴数据限幅值
#define YAW_ABS_MAX 3.14f
#define YAW_ABS_MIN -3.14f
#define YAW_REL_MAX 3.14f
#define YAW_REL_MIN -3.14f
//pitch轴数据限幅值
#define PITCH_ABS_MAX 3.14f
#define PITCH_ABS_MIN -3.14f
#define PITCH_REL_MAX 3.14f
#define PITCH_REL_MIN -3.14f


#define YAW_POS_P 8.0
#define YAW_POS_I 0.0
#define YAW_POS_D 2.0
#define YAW_POS_MAX_OUT 10.0
#define YAW_POS_MIN_OUT -10.0
#define YAW_POS_MAX_IOUT 0
#define YAW_POS_MIN_IOUT 0

#define YAW_OMEGA_P 8.0
#define YAW_OMEGA_I 0.0
#define YAW_OMEGA_D 2.0
#define YAW_OMEGA_MAX_OUT 10.0
#define YAW_OMEGA_MIN_OUT -10.0
#define YAW_OMEGA_MAX_IOUT 0
#define YAW_OMEGA_MIN_IOUT 0

#define PITCH_POS_P 8.0
#define PITCH_POS_I 0
#define PITCH_POS_D 2.0
#define PITCH_POS_MAX_OUT 10.0
#define PITCH_POS_MIN_OUT -10.0
#define PITCH_POS_MAX_IOUT 0
#define PITCH_POS_MIN_IOUT 0


typedef enum
{
    MOTOR_INIT = 0,
    MOTOR_GYRO = 1,
    MOTOR_ENCODER = 2,
}motor_mode_enum;
//单个欧拉角结构体
typedef struct
{
    motor_mode_enum motorMode;
    pid_struct euler_pos_control;//角度环
    pid_struct euler_omega_control;//角速度环

    float absolute_angle;
    float absolute_angle_set;
    float absolute_angle_max;
    float absolute_angle_min;

    float relative_angle;
    float relative_angle_set;
    float relative_angle_max;
    float relative_angle_min;
}gimbal_motor_struct;
typedef struct
{
    // 扫描低通滤波结构体
    first_order_filter_type_t pitch_auto_scan_first_order_filter;
    first_order_filter_type_t yaw_auto_scan_first_order_filter;

    //yaw轴中心值
    float yaw_center_value;
    //pitch轴中心值
    float pitch_center_value;

    //yaw轴运动幅度
    float yaw_range;
    //pitch轴运动幅度
    float pitch_range;

    //当前运行时间 单位s
    float scan_run_time;
    //初始计时时间 单位s
    float scan_begin_time;

    //yaw轴扫描周期 单位s
    float scan_yaw_period;
    //pitch轴扫描周期  单位s
    float scan_pitch_period;

}scan_struct;
typedef struct
{
    INS_t *imu_point;//陀螺仪数据
    //radar_data_struct *radar_point;
    vision_data_struct *vision_point;//视觉数据
    rc_ctrl_struct *rc_point;//遥控器数据

    gimbal_motor_struct yawEuler;
    dm_control_struct yawMotor;//yaw电机结构体
    gimbal_motor_struct pitchEuler;
    dm_control_struct pitchMotor;//pitch电机结构体

    scan_struct gimbalScan;//自动扫描

    bool enable;

    HAL_StatusTypeDef can_tx_status;
    uint8_t can_tx_data[8];
}gimbal_control_struct;

extern gimbal_control_struct gimbalControl;//云台控制

#endif //RADAR_GIMBAL_H7_GIMBAL_TASK_H