//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_behaviour.h"

gimbal_mode_enum gimbalMode;//ÔÆÌ¨Ä£Ê½


void gimbal_behaviour_update(gimbal_control_struct *mode)
{
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
            // if ()
            // {
            //
            // }
            break;
        }
        case GIMBAL_MANUAL_MODE:
        {
            break;
        }
        case GIMBAL_AUTO_MODE:
        {
            break;
        }
        case GIMBAL_INABILITY_MODE:
        {
            break;
        }
    }
}
void gimbal_behaviour_mode_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_get)
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
}