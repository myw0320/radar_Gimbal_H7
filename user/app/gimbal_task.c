//
// Created by myw04 on 2025/11/8.
//

#include "gimbal_task.h"
#include "gimbal_behaviour.h"


gimbal_control_struct gimbalControl;//云台控制

static void gimbal_init(gimbal_control_struct *init);
static void gimbal_update(gimbal_control_struct *update);
static void gimbal_control_set(gimbal_control_struct *control);
static void gimbal_control(gimbal_control_struct *control);
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_struct *motor_control);
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_struct *motor_control);


/******/
const float yaw_omega_k[3] = {YAW_OMEGA_P,YAW_OMEGA_I,YAW_OMEGA_D};
const float yaw_pos_k[3] = {YAW_POS_P,YAW_POS_I,YAW_POS_D};
const float pitch_omega_k[3] = {};
const float pitch_pos_k[3] = {};
void gimbal_task(void *pvParameters)
{
    //云台初始化
    gimbal_init(&gimbalControl);
    while(1)
    {
        //
        gimbal_update(&gimbalControl);//更新原始数据
        //更新控制数据
        gimbal_control_set(&gimbalControl);//设置
        gimbal_control(&gimbalControl);//计算控制值
        //发送电流值
        if (!toe_is_error(GIMBAL_YAW_TOE) && !toe_is_error(GIMBAL_PITCH_TOE))
        {
            //电机在线发送电流值
            DM_AddTxPacket(0xFE,gimbalControl.yawMotor.give_cmd_current,gimbalControl.pitchMotor.give_cmd_current,0,0);
        }
    }
}


static void gimbal_init(gimbal_control_struct *init)
{
    //获取指针地址
    init->imu_point = &INS;
    init->vision_point = &visionData;
    init->radar_point = &radarData;
    init->rc_point = &rcData;


    //yaw数据初始化
    DM_Init(&init->yawMotor,DM4310,0x01);//电机控制初始化
    init->yawEuler.absolute_angle_max = YAW_ABS_MAX;
    init->yawEuler.absolute_angle_min = YAW_ABS_MIN;
    init->yawEuler.relative_angle_max = YAW_REL_MAX;
    init->yawEuler.relative_angle_min = YAW_REL_MIN;
    PID_init(&init->yawEuler.euler_omega_control,PID_POSITION,yaw_omega_k,10,0);//
    PID_init(&init->yawEuler.euler_pos_control,PID_POSITION,yaw_pos_k,10,10);
    //pitch数据初始化
    DM_Init(&init->pitchMotor,DM4310,0x02);//电机控制初始化
    init->pitchEuler.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEuler.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEuler.relative_angle_max = PITCH_REL_MAX;
    init->pitchEuler.relative_angle_min = PITCH_REL_MIN;
    PID_init(&init->pitchEuler.euler_omega_control,PID_POSITION,pitch_omega_k,10,0);//
    PID_init(&init->pitchEuler.euler_pos_control,PID_POSITION,pitch_pos_k,10,10);
}


static float motor_ecd_to_rad(uint16_t ecd, uint16_t zero_ecd)
{
    uint16_t relative_ecd = ecd - zero_ecd;

    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }
    return (float)relative_ecd * MOTOR_ECD_TO_RAD;
}
//数据更新
static void gimbal_update(gimbal_control_struct *update)
{
    //yaw数据更新
    DM_RxPacketUpdate(&update->yawMotor);
    update->yawEuler.absolute_angle = update->imu_point->Yaw;//获取绝对角
    update->yawEuler.relative_angle = motor_ecd_to_rad(update->yawMotor.motor_measurement.encoder,YAW_ZERO_ENCODER);//获取相对角
    //pitch数据更新
    DM_RxPacketUpdate(&update->pitchMotor);
    update->pitchEuler.absolute_angle = update->imu_point->Pitch;//获取绝对角
    update->pitchEuler.relative_angle = motor_ecd_to_rad(update->pitchMotor.motor_measurement.encoder,PITCH_ZERO_ENCODER);//获取相对角
}


static void gimbal_control_update(gimbal_control_struct)
{

}
//控制设置
static void gimbal_control_set(gimbal_control_struct *control)
{
    //更新云台各电机行为
    gimbal_motor_mode_update(control);
    switch (control->yawEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            break;
        }
        case MOTOR_GYRO:
        {
            gimbal_motor_gyro_control_set(control,&control->yawEuler);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control_set(control,&control->yawEuler);
            break;
        }
        default:
        {
            break;
        }
    }
    switch (control->pitchEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            break;
        }
        case MOTOR_GYRO:
        {
            gimbal_motor_gyro_control_set(control,&control->pitchEuler);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control_set(control,&control->pitchEuler);
            break;
        }
        default:
        {
            break;
        }
    }
}

static void gimbal_control(gimbal_control_struct *control)
{
    switch (control->yawEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            break;
        }
        case MOTOR_GYRO:
        {
            gimbal_motor_gyro_control(control,&control->yawEuler);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control(control,&control->yawEuler);
            break;
        }
        default:
        {
            break;
        }
    }
    switch (control->pitchEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            break;
        }
        case MOTOR_GYRO:
        {
            gimbal_motor_gyro_control(control,&control->pitchEuler);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control(control,&control->pitchEuler);
            break;
        }
        default:
        {
            break;
        }
    }
}
static void gimbal_motor_encoder_control_set()
{

}
static void gimbal_motor_gyro_control_set()
{

}
//电机编码值控制
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_struct *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    if (motor_control == &gimbal_control->yawEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        PID_calc(&motor_control->euler_omega_control,gimbal_control->yawMotor.omega,motor_control->euler_omega_control.out);
        gimbal_control->yawMotor.give_cmd_current = (int16_t)motor_control->euler_omega_control.out;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        PID_calc(&motor_control->euler_omega_control,gimbal_control->pitchMotor.omega,motor_control->euler_omega_control.out);
        gimbal_control->pitchMotor.give_cmd_current = (int16_t)motor_control->euler_omega_control.out;
    }
}

//电机陀螺仪控制
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_struct *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    if (motor_control == &gimbal_control->yawEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        PID_calc(&motor_control->euler_omega_control,gimbal_control->yawMotor.omega,motor_control->euler_omega_control.out);
        gimbal_control->yawMotor.give_cmd_current = (int16_t)motor_control->euler_omega_control.out;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        PID_calc(&motor_control->euler_omega_control,gimbal_control->pitchMotor.omega,motor_control->euler_omega_control.out);
        gimbal_control->pitchMotor.give_cmd_current = (int16_t)motor_control->euler_omega_control.out;
    }
}