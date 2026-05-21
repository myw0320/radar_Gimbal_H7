#ifndef __H7_CALIBRATE_TASK_H
#define __H7_CALIBRATE_TASK_H
#include "stdint.h"
#include "stdbool.h"
#include "dt7.h"


typedef enum
{
    GIMBAL_CALI_IDLE = 0,
    GIMBAL_CALI_TO_ENCODER_ZERO,
    GIMBAL_CALI_DONE,
    GIMBAL_CALI_ERROR,
} gimbal_cali_state_e;


typedef struct
{
    bool gimbal_cali_flag;//云台归为标志位
    bool imu_start_flag;//imu开始
}cali_flag_t;
#endif //__H7_CALIBRATE_TASK_H