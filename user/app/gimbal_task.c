//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_task.h"



gimbal_control_struct gimbalControl;//云台控制

static void gimbal_init(gimbal_control_struct *init);
static void gimbal_update(gimbal_control_struct *update);

void gimbal_task(void)
{
    //云台初始化
    gimbal_init(&gimbalControl);
    while(1)
    {
        gimbal_mode_update(&gimbalControl);

        DM_AddTxPacket(0xFE,gimbalControl.yawMotor.give_cmd_current,gimbalControl.pitchMotor.give_cmd_current,0,0);
    }
}


static void gimbal_init(gimbal_control_struct *init)
{
    //获取指针地址
    init->imuData = &INS;
    init->visionData = &visionData;



    //yaw数据初始化
    DM_Init(&init->yawMotor,DM4310,0x01);//电机控制初始化
    init->yawEulerAngle.absolute_angle_max = YAW_ABS_MAX;
    init->yawEulerAngle.absolute_angle_min = YAW_ABS_MIN;
    init->yawEulerAngle.relative_angle_max = YAW_REL_MAX;
    init->yawEulerAngle.relative_angle_min = YAW_REL_MIN;
    //pitch数据初始化
    DM_Init(&init->pitchMotor,DM4310,0x02);//电机控制初始化
    init->pitchEulerAngle.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEulerAngle.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEulerAngle.relative_angle_max = PITCH_REL_MAX;
    init->pitchEulerAngle.relative_angle_min = PITCH_REL_MIN;
}


static float motor_ecd_to_rad(uint16_t ecd, uint16_t zero_ecd)
{
    float relative_ecd = ecd - zero_ecd;

    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
//数据更新
void gimbal_update(gimbal_control_struct *update)
{
    //yaw数据更新
    DM_RxPacketUpdate(&update->yawMotor);
    update->yawEulerAngle.absolute_angle = update->imuData->Yaw;
    update->yawEulerAngle.relative_angle = motor_ecd_to_rad(update->yawMotor.motor_measurement.encoder,YAW_ZERO_ENCODER);
    //pitch数据更新
    DM_RxPacketUpdate(&update->pitchMotor);
    update->pitchEulerAngle.absolute_angle = update->imuData->Pitch;
    update->pitchEulerAngle.relative_angle = motor_ecd_to_rad(update->pitchMotor.motor_measurement.encoder,PITCH_ZERO_ENCODER);
}
//
void gimbal_init_control()
{
    //云台初始化
}
//手动控制
void gimbal_inability_control()
{

}
void gimbal_auto_control()
{

}
//
void gimbal_control_set(gimbal_control_struct *control)
{
    switch (control->yawEulerAngle.motorMode)
    {
        case MOTOR_INIT:

            break;
        case MOTOR_GYRO:
            break;
        case MOTOR_ENCODER:
            break;
    }
    switch (control->pitchEulerAngle.motorMode)
    {
        case MOTOR_INIT:
            break;
        case MOTOR_GYRO:
            break;
        case MOTOR_ENCODER:
            break;
    }
    // switch(gimbalMode)
    // {
    //     case GIMBAL_INIT_MODE:
    //     {
    //         break;
    //     }
    //     case GIMBAL_MANUAL_MODE:
    //     {
    //         break;
    //     }
    //     case GIMBAL_AUTO_MODE:
    //     {
    //         break;
    //     }
    //     case GIMBAL_INABILITY_MODE:
    //     {
    //         gimbal_inability_control();
    //         break;
    //     }
    // }
}