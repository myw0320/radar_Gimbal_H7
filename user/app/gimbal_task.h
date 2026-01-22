
#ifndef RADAR_GIMBAL_H7_GIMBAL_TASK_H
#define RADAR_GIMBAL_H7_GIMBAL_TASK_H
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "ins_task.h"
#include "vision_task.h"
#include "radar_task.h"
#include "user_lib.h"
#include "fsi6.h"
#include "dt7.h"
#include "Dji_motor.h"
#include "DM_motor.h"
#include "pid.h"
#include "Mathh.h"


//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)


#define ECD_RANGE 8191
#define HALF_ECD_RANGE 4096
#define MOTOR_ECD_TO_RAD 0.000766990394f //编码值转换为弧度值  2*PI/8192
//yaw电机零点编码值
#define YAW_ZERO_ENCODER 4096
//pitch电机零点编码值
#define PITCH_ZERO_ENCODER 4096
//yaw轴数据限幅值
#define YAW_ABS_MAX 0.5f
#define YAW_ABS_MIN -0.5f
#define YAW_REL_MAX 3.14f
#define YAW_REL_MIN -3.14f
//pitch轴数据限幅值
#define PITCH_ABS_MAX 0.33f
#define PITCH_ABS_MIN -0.5f
#define PITCH_REL_MAX 3.14f
#define PITCH_REL_MIN -3.14f

#define YAW_POS_P 10.0f
#define YAW_POS_I 0.0
#define YAW_POS_D 4.0f
#define YAW_POS_MAX_OUT 6.0
#define YAW_POS_MIN_OUT -12.0
#define YAW_POS_MAX_IOUT 0
#define YAW_POS_MIN_IOUT 0

#define PITCH_POS_P 10.0f
#define PITCH_POS_I 0
#define PITCH_POS_D 4.0f
#define PITCH_POS_MAX_OUT 12.0
#define PITCH_POS_MIN_OUT -12.0
#define PITCH_POS_MAX_IOUT 0
#define PITCH_POS_MIN_IOUT 0

#define X_ERR_MAX_OUT 0.5f
#define X_ERR_MIN_OUT -0.5f
#define X_ERR_MAX_IOUT 0.2f
#define X_ERR_MIN_IOUT -0.2f

#define Y_ERR_MAX_OUT 0.5f
#define Y_ERR_MIN_OUT -0.5f
#define Y_ERR_MAX_IOUT 0.2f
#define Y_ERR_MIN_IOUT -0.2f
typedef enum
{
    MOTOR_INIT = 0,
    MOTOR_GYRO,
    MOTOR_ENCODER,
    MOTOR_STOP
}motor_mode_enum;
//单个欧拉角结构体
typedef struct
{
    motor_mode_enum motorMode;
    motor_mode_enum last_motorMode;

    pid_struct euler_pos_control;//角度环
    pid_struct euler_omega_control;//角速度环

    float absolute_zero_angle;
    float absolute_angle;
    float absolute_angle_set;
    float absolute_angle_max;
    float absolute_angle_min;

    float relative_angle;
    float relative_angle_set;
    float relative_angle_max;
    float relative_angle_min;
}gimbal_motor_t;

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
    pid_struct x_err_pid;
    pid_struct y_err_pid;

    float target_x;//x设定值
    float target_y;
    float now_x;
    float now_y;
    //低通滤波器
    first_order_filter_type_t x_LFT;
    first_order_filter_type_t y_LFT;
    //时间
    float current_time;
}vision_to_gimbal_t;


typedef struct
{
    const INS_t *imu_point;//陀螺仪数据
    radar_data_t *radar_point;
    vision_data_t *vision_point;//视觉数据
    const dt7_data_struct *rc_point;//遥控器数据
    vision_to_gimbal_t visionToGimbal;
    gimbal_motor_t yawEuler;
    dm_control_t yawMotor;//yaw电机结构体
    gimbal_motor_t pitchEuler;
    dm_control_t pitchMotor;//pitch电机结构体

    scan_struct gimbalScan;//自动扫描

    bool enable;

    HAL_StatusTypeDef can_tx_status;
    uint8_t can_tx_data[8];
}gimbal_control_struct;

extern gimbal_control_struct gimbalControl;//云台控制

#endif //RADAR_GIMBAL_H7_GIMBAL_TASK_H