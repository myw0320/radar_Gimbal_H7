//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_task.h"
gimbal_mode_enum gimbalMode;//云台模式
gimbal_control_struct gimbalControl;//云台控制

static void gimbal_init(gimbal_control_struct *init);


void gimbal_task(void)
{
    //云台初始化
    gimbal_init(&gimbalControl);
    while(1)
    {


    }
}


static void gimbal_init(gimbal_control_struct *init)
{
    gimbalMode = GIMBAL_INIT_MODE;
    //init->visionData = &visionData;
}

void gimbal_mode_update(gimbal_control_struct *mode)
{
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
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

//数据更新
void gimbal_update(gimbal_control_struct *update)
{

}
void gimbal_init_control()
{

}
//手动控制
void gimbal_inability_control()
{

}
void gimbal_auto_control()
{

}
//
void gimbal_control(gimbal_control_struct *control)
{
    switch(gimbalMode)
    {
        case GIMBAL_INIT_MODE:
        {
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