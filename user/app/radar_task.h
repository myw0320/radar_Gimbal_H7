#ifndef RADAR_GIMBAL_H7_RADAR_TASK_H
#define RADAR_GIMBAL_H7_RADAR_TASK_H
#include "stdint.h"
typedef struct __attribute__((packed))
{
    float x_error;
    float y_error;

    float current_receive_time;
}radar_data_struct;

extern radar_data_struct radarData;


void radar_to_gimbal(float *yaw,float *pitch,radar_data_struct *gimbal_control);
#endif //RADAR_GIMBAL_H7_RADAR_TASK_H