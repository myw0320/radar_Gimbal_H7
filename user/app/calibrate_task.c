#include "calibrate_task.h"
#include "gimbal_task.h"


cali_flag_t cali_flag;

void gimbal_cali(cali_flag_t *gimbal_cali);
//整机校准
void Calibrate_Task(void const *pvParameters)
{

}


void gimbal_cali(cali_flag_t *gimbal_cali)
{
    gimbalControl.yawEuler.relative_angle_set = 0;
    gimbalControl.pitchEuler.relative_angle_set = 0;

    gimbal_motor_encoder_control(&gimbalControl,&gimbalControl.yawEuler);
    gimbal_motor_encoder_control(&gimbalControl,&gimbalControl.pitchEuler);
    if (fabs(gimbalControl.yawMotor.motor_measurement.pos) < 0.1 &&
        fabs(gimbalControl.pitchMotor.motor_measurement.pos < 0.1))
    {
        gimbal_cali->imu_start_flag = 1;
        gimbal_cali->gimbal_cali_flag = 0;
    }
    else
    {
        gimbal_cali->imu_start_flag = 0;
        gimbal_cali->gimbal_cali_flag = 1;
    }
}