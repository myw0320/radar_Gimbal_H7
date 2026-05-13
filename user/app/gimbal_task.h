
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
#define YAW_ABS_ZERO 0.0f
#define YAW_ABS_MAX 0.6f
#define YAW_ABS_MIN -0.6f
#define YAW_REL_ZERO 0.0f
#define YAW_REL_MAX 3.14f
#define YAW_REL_MIN -3.14f
//pitch轴数据限幅值
#define PITCH_ABS_ZERO 0.0f
#define PITCH_ABS_MAX 0.36f
#define PITCH_ABS_MIN -0.43f
#define PITCH_REL_ZERO 0.0f
#define PITCH_REL_MAX 3.14f
#define PITCH_REL_MIN -3.14f

#define YAW_POS_P 30.0f
#define YAW_POS_I 0.065f
#define YAW_POS_D 2.0f
#define YAW_POS_MAX_OUT 20.0
#define YAW_POS_MIN_OUT -20.0
#define YAW_POS_MAX_IOUT 5.0
#define YAW_POS_MIN_IOUT -5.0

#define PITCH_POS_P 30.0f
#define PITCH_POS_I 0.07f
#define PITCH_POS_D 2.0f
#define PITCH_POS_MAX_OUT 20.0
#define PITCH_POS_MIN_OUT -20.0
#define PITCH_POS_MAX_IOUT 5.0
#define PITCH_POS_MIN_IOUT -5.0

#define X_ERR_MAX_OUT 15.0f
#define X_ERR_MIN_OUT -15.0f
#define X_ERR_MAX_IOUT 0.50f
#define X_ERR_MIN_IOUT -0.5f

#define Y_ERR_MAX_OUT 10.0f
#define Y_ERR_MIN_OUT -10.0f
#define Y_ERR_MAX_IOUT 0.5f
#define Y_ERR_MIN_IOUT -0.5f
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

    float vel_set;

    float absolute_zero_angle;
    float absolute_angle;
    float absolute_angle_set;
    float absolute_angle_max;
    float absolute_angle_min;

    float relative_zero_angle;
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

    // 自动扫描设置浮动值
    float auto_scan_AC_set_yaw;
    float auto_scan_AC_set_pitch;

    int8_t last_yaw_dir;
    float pitch_accumulated; // 用于记录蛇形扫描中 Pitch 轴当前走到了哪里
}scan_struct;




typedef struct
{
    const INS_t *imu_point;//陀螺仪数据
    const dt7_data_struct *rc_dt7_point;//遥控器数据
    const fsi6_data_struct *rc_fsi6_point;

    radar_data_t *radar_point;
    vision_data_t *vision_point;//视觉数据


    gimbal_motor_t yawEuler;
    dm_control_t yawMotor;//yaw电机结构体
    gimbal_motor_t pitchEuler;
    dm_control_t pitchMotor;//pitch电机结构体

    scan_struct gimbalScan;//自动扫描
    bool enable;

    uint8_t yaw_can_tx_data[8];
    uint8_t pitch_can_tx_data[8];
}gimbal_control_struct;

extern gimbal_control_struct gimbalControl;//云台控制
void UserGimbal_AddTxPacket(void);
#endif //RADAR_GIMBAL_H7_GIMBAL_TASK_H