//
// Created by myw04 on 2025/11/8.
//

#ifndef RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#define RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H
#include "main.h"
#include "gimbal_task.h"
#include "controller.h"
#include "user_lib.h"

/* 宏定义 ------------------------------------------------------------------ */

// 遥控器摇杆角度分辨率
#define YAW_RC_SEN  0.000001f
#define PITCH_RC_SEN  0.000001f

// 鼠标灵敏度
#define YAW_MOUSE_SEN  0.00005f
#define PITCH_MOUSE_SEN  0.00005f

// 时间 ms → s
#define TIME_MS_TO_S(ms) (float)(ms / 1000.0f)

// 雷达到位判定阈值
#define RADAR_REACHED_YAW_ERR 0.05f
#define RADAR_REACHED_PITCH_ERR 0.06f

// 自动扫描参数
#define AUTO_SCAN_YAW_RANGE 0.20f
#define AUTO_SCAN_PITCH_RANGE 0.20f
#define AUTO_SCAN_YAW_PERIOD 1.5f
#define AUTO_SCAN_PITCH_STEP 0.04f

/* 枚举定义 ---------------------------------------------------------------- */

/** 云台控制模式（遥控器开关决定） */
typedef enum
{
    GIMBAL_STOP = 0,    // 停止
    GIMBAL_INIT,        // 初始化
    GIMBAL_AUTO,        // 自动
    GIMBAL_MANUAL,      // 手动
}gimbal_control_mode_enum;

/** 云台子模式（由控制模式 + 数据源细分） */
typedef enum
{
    GIMBAL_NO_MOVE_MODE = 0,       // 静止
    GIMBAL_INIT_MODE = 1,          // 初始化

    GIMBAL_MANUAL_MOVE_MODE = 2,   // 手动移动
    GIMBAL_MANUAL_ATTACK_MODE = 3, // 手动攻击

    GIMBAL_AUTO_MOVE_MODE = 4,     // 自动移动（雷达引导）
    GIMBAL_AUTO_SCAN_MODE = 5,     // 自动扫描
    GIMBAL_AUTO_ATTACK_MODE = 6,   // 自动攻击（视觉引导）
}gimbal_mode_enum;

/* 全局变量声明 ------------------------------------------------------------- */

extern gimbal_mode_enum gimbalMode; // 云台当前子模式

/* 函数声明 ---------------------------------------------------------------- */

void gimbal_motor_mode_update(gimbal_control_struct *motor_mode_update);
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_struct *gimbal_control_set);

#endif //RADAR_GIMBAL_GIMBAL_BEHAVIOUR_H