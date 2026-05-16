#include "calibrate_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include <math.h>

#define GIMBAL_ZERO_CONTROL_KP          22.0f
#define GIMBAL_ZERO_CONTROL_KD          1.0f
#define GIMBAL_ZERO_TARGET              0.0f
#define GIMBAL_ZERO_POS_ERR             0.025f
#define GIMBAL_ZERO_VEL_ERR             0.035f
#define GIMBAL_ZERO_STABLE_TIME_MS      500U
#define GIMBAL_ZERO_TIMEOUT_MS          6000U

typedef enum
{
    GIMBAL_CALI_IDLE = 0,
    GIMBAL_CALI_TO_ENCODER_ZERO,
    GIMBAL_CALI_DONE,
    GIMBAL_CALI_ERROR,
} gimbal_cali_state_e;

cali_flag_t cali_flag = {
    .gimbal_cali_flag = 1,
    .imu_start_flag = 0,
};

float gimbal_yaw_center_offset = 0.0f;
static gimbal_cali_state_e gimbal_cali_state = GIMBAL_CALI_IDLE;



void Calibrate_Task(void const *pvParameters)
{
    //Cali_Task(pvParameters);
}
