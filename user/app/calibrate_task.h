#ifndef __H7_CALIBRATE_TASK_H
#define __H7_CALIBRATE_TASK_H
#include "stdint.h"

#include "dt7.h"

//cali device name
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    CALI_MAG = 4,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;



//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    float pitch_min_angle;
} gimbal_cali_t;

//gyro, accel, mag device
typedef struct
{
    float offset[3]; //x,y,z
    float scale[3];  //x,y,z
} imu_cali_t;


#endif //__H7_CALIBRATE_TASK_H