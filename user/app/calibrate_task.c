#include "calibrate_task.h"

static dt7_data_struct *cali_rc_dt7;    //remote control point
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
//整机校准
void Calibrate_Task(void const *pvParameters)
{

    for (int i =0;i<CALI_LIST_LENGHT;i++)
    {

    }
}

//遥控器控制校准
void rc_cali(void)
{

}


void gimbal_cali()
{

}