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
#include "cap_comm.h"
#include "PID_control.h"
#include "Filter_control.h"

/* 宏定义 ------------------------------------------------------------------ */

// 角度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#define ECD_RANGE 8191
#define HALF_ECD_RANGE 4096
// 编码值转换为弧度值  2*PI/8192
#define MOTOR_ECD_TO_RAD 0.000766990394f

// yaw电机零点编码值
#define YAW_ZERO_ENCODER 4096
// pitch电机零点编码值
#define PITCH_ZERO_ENCODER 4096

// yaw轴角度限幅值
#define YAW_ABS_ZERO 0.0f
#define YAW_ABS_MAX 0.6f
#define YAW_ABS_MIN -0.6f
#define YAW_REL_ZERO 0.0f
#define YAW_REL_MAX 3.14f
#define YAW_REL_MIN -3.14f

// pitch轴角度限幅值
#define PITCH_ABS_ZERO 0.0f
#define PITCH_ABS_MAX 0.36f
#define PITCH_ABS_MIN -0.43f
#define PITCH_REL_ZERO 0.0f
#define PITCH_REL_MAX 3.14f
#define PITCH_REL_MIN -3.14f

#define GIMBAL_PID_PERIOD 0.001f

// yaw轴绝对位置PID参数
#define YAW_ABS_POS_P 42.0f
#define YAW_ABS_POS_I 0.25f
#define YAW_ABS_POS_D 1.0f
#define YAW_ABS_POS_F 0.4f
#define YAW_ABS_POS_MAX_OUT 10.0
#define YAW_ABS_POS_MIN_OUT -10.0
#define YAW_ABS_POS_MAX_IOUT 5.0
#define YAW_ABS_POS_MIN_IOUT -5.0

// yaw轴相对位置PID参数
#define YAW_REL_POS_P 15.0f
#define YAW_REL_POS_I 0.0f
#define YAW_REL_POS_D 2.0f
#define YAW_REL_POS_F 0.0f
#define YAW_REL_POS_MAX_OUT 2.0f
#define YAW_REL_POS_MIN_OUT -2.0f
#define YAW_REL_POS_MAX_IOUT 1.0f
#define YAW_REL_POS_MIN_IOUT -1.0f

// pitch轴绝对位置PID参数
#define PITCH_ABS_POS_P 48.0f
#define PITCH_ABS_POS_I 0.15f
#define PITCH_ABS_POS_D 5.0f
#define PITCH_ABS_POS_F 0.4f
#define PITCH_ABS_POS_MAX_OUT 10.0f
#define PITCH_ABS_POS_MIN_OUT -10.0f
#define PITCH_ABS_POS_MAX_IOUT 5.0f
#define PITCH_ABS_POS_MIN_IOUT -5.0f

// pitch轴相对位置PID参数
#define PITCH_REL_POS_P 20.0f
#define PITCH_REL_POS_I 0.025f
#define PITCH_REL_POS_D 5.0f
#define PITCH_REL_POS_F 0.0f
#define PITCH_REL_POS_MAX_OUT 5.0
#define PITCH_REL_POS_MIN_OUT -5.0
#define PITCH_REL_POS_MAX_IOUT 1.0
#define PITCH_REL_POS_MIN_IOUT -1.0

/* 枚举与结构体 ------------------------------------------------------------- */

/** 电机控制模式 */
typedef enum
{
    MOTOR_INIT = 0,     // 初始化模式
    MOTOR_GYRO,         // 陀螺仪（绝对角）模式
    MOTOR_ENCODER,      // 编码器（相对角）模式
    MOTOR_STOP          // 停止模式
}motor_mode_enum;

/** 云台欧拉角结构体 */
typedef struct
{
    motor_mode_enum motorMode;
    motor_mode_enum last_motorMode;

    pid_ctrl_t euler_abs_pos_control;   // 绝对角PID
    pid_ctrl_t euler_rel_pos_control;   // 相对角PID
    pid_ctrl_t euler_vel_control;       // 速度PID

    filter_ctrl_t euler_filter;         // 滤波器
    float vel_set;                      // 速度设定值
    float vel;                          // 速度反馈值

    float absolute_zero_angle;          // 绝对角零点
    float absolute_angle;               // 绝对角当前值
    float absolute_angle_set;           // 绝对角设定值
    float absolute_angle_max;           // 绝对角上限
    float absolute_angle_min;           // 绝对角下限

    float relative_zero_angle;          // 相对角零点
    float relative_angle;               // 相对角当前值
    float relative_angle_set;           // 相对角设定值
    float relative_angle_max;           // 相对角上限
    float relative_angle_min;           // 相对角下限
}gimbal_motor_t;

/** 自动扫描结构体 */
typedef struct
{
    first_order_filter_type_t pitch_auto_scan_first_order_filter;
    first_order_filter_type_t yaw_auto_scan_first_order_filter;

    float yaw_center_value;             // yaw轴扫描中心值
    float pitch_center_value;           // pitch轴扫描中心值

    float yaw_range;                    // yaw轴扫描范围
    float pitch_range;                  // pitch轴扫描范围

    float scan_run_time;                // 当前扫描运行时间 (s)
    float scan_begin_time;              // 扫描起始时间 (s)

    float scan_yaw_period;              // yaw轴扫描周期 (s)
    float scan_pitch_period;            // pitch轴扫描周期 (s)

    float auto_scan_AC_set_yaw;         // 自动扫描yaw给定值
    float auto_scan_AC_set_pitch;       // 自动扫描pitch给定值

    int8_t last_yaw_dir;                // 上一周期yaw扫描方向
    float pitch_accumulated;            // pitch轴自动扫描角度累计值
}scan_struct;

/** 云台总控结构体 */
typedef struct
{
    const INS_t *imu_point;             // IMU数据指针
    const dt7_data_struct *rc_dt7_point;// DT7遥控器数据指针
    const fsi6_data_struct *rc_fsi6_point;// FSi6遥控器数据指针

    radar_data_t *radar_point;          // 雷达数据指针
    vision_data_t *vision_point;        // 视觉数据指针

    gimbal_motor_t yawEuler;            // yaw轴欧拉角控制
    dm_control_t yawMotor;              // yaw轴电机
    gimbal_motor_t pitchEuler;          // pitch轴欧拉角控制
    dm_control_t pitchMotor;            // pitch轴电机

    scan_struct gimbalScan;             // 自动扫描数据
    bool enable;

    float current_time;

    uint8_t yaw_can_tx_data[8];         // yaw轴CAN发送缓存
    uint8_t pitch_can_tx_data[8];       // pitch轴CAN发送缓存
}gimbal_control_struct;

/* 全局变量声明 ------------------------------------------------------------- */

extern gimbal_control_struct gimbalControl; // 云台控制句柄

/* 函数声明 ---------------------------------------------------------------- */

void UserGimbal_AddTxPacket(void);

#endif //RADAR_GIMBAL_H7_GIMBAL_TASK_H