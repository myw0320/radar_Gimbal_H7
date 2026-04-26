#include "hipnuc_imu_task.h"

#include "cmsis_os.h"
#include "hipnuc.h"

hi04_imu_data_t hi04_imu_data;
void HIPNUC_Init(void)
{
    hi04_init(&hi04_imu_data);
}
void HIPNUC_Task(void const * argument)
{

    while (1)
    {
        Hi04_GetRxPacket(&hi04_imu_data);
        osDelay(1);
    }
}