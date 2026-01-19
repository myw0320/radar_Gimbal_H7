#include "gimbal_task.h"
#include "can_comm_task.h"
#include "gimbal_behaviour.h"
#include "ws2812.h"
#include "cmsis_os.h"
#include "cap_comm.h"

gimbal_control_struct gimbalControl;//云台控制

static void gimbal_init(gimbal_control_struct *init);
static void gimbal_update(gimbal_control_struct *update);
static void gimbal_update_save(gimbal_control_struct *save);
static void gimbal_control_set(gimbal_control_struct *control);
static void gimbal_control(gimbal_control_struct *control);
static void gimbal_motor_encoder_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
static void gimbal_motor_gyro_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);

//24.03mm
/******/
const float yaw_pos_k[3] = {YAW_POS_P,YAW_POS_I,YAW_POS_D};
const float pitch_pos_k[3] = {PITCH_POS_P,PITCH_POS_I,PITCH_POS_D};

const float x_err_k[3] = {10.0f,0.0f,4.0f};
const float y_err_k[3] = {10.0f,0.0f,4.0f};
cap_tx_data_t cap_tx_test;

void Gimbal_Task(void const *pvParameters)
{
    //初始化
    gimbal_init(&gimbalControl);
    while(1)
    {
        gimbal_update(&gimbalControl);//更新数据
        gimbal_control_set(&gimbalControl);//设置
        gimbal_update_save(&gimbalControl);
        gimbal_control(&gimbalControl);//计算控制值

        //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
        // CAP_AddTxPacket(&cap_tx_test,gimbalControl.can_tx_data);
        // can_tx_data(&hfdcan1,0x210,gimbalControl.can_tx_data,8);
        if (gimbalControl.yawMotor.motor_measurement.state == 1)
        {
            DM_AddTxPacket(&gimbalControl.yawMotor,gimbalControl.can_tx_data);
            gimbalControl.can_tx_status = can_tx_data(&hfdcan1,0x201,gimbalControl.can_tx_data,8);
        }
        else
        {
            DM_Enable(gimbalControl.can_tx_data);
            can_tx_data(&hfdcan1,0x201,gimbalControl.can_tx_data,8);
        }

        if (gimbalControl.pitchMotor.motor_measurement.state == 1)
        {
            DM_AddTxPacket(&gimbalControl.pitchMotor,gimbalControl.can_tx_data);
            gimbalControl.can_tx_status = can_tx_data(&hfdcan2,0x202,gimbalControl.can_tx_data,8);
        }
        else
        {
            DM_Enable(gimbalControl.can_tx_data);
            can_tx_data(&hfdcan2,0x202,gimbalControl.can_tx_data,8);
        }
        osDelay(2);
    }
}


static void gimbal_init(gimbal_control_struct *init)
{
    //获取指针地址
    init->imu_point = &INS;
    init->vision_point = &visionData;
    //init->radar_point = &radarData;
    init->rc_point = &dt7Data;

    //yaw数据初始化
    DM_Init(&init->yawMotor,DM4310,OMEGA,0x01);//电机控制初始化
    init->yawEuler.absolute_angle_max = YAW_ABS_MAX;
    init->yawEuler.absolute_angle_min = YAW_ABS_MIN;
    init->yawEuler.relative_angle_max = YAW_REL_MAX;
    init->yawEuler.relative_angle_min = YAW_REL_MIN;
    PID_init(&init->yawEuler.euler_pos_control,PID_POSITION,yaw_pos_k,YAW_POS_MAX_OUT,YAW_POS_MIN_OUT,YAW_POS_MAX_IOUT,YAW_POS_MIN_IOUT);
    //pitch数据初始化
    DM_Init(&init->pitchMotor,DM4310,OMEGA,0x06);//电机控制初始化
    init->pitchEuler.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEuler.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEuler.relative_angle_max = PITCH_REL_MAX;
    init->pitchEuler.relative_angle_min = PITCH_REL_MIN;
    PID_init(&init->pitchEuler.euler_pos_control,PID_POSITION,pitch_pos_k,PITCH_POS_MAX_OUT,PITCH_POS_MIN_OUT,PITCH_POS_MAX_IOUT,PITCH_POS_MIN_IOUT);
    //初始化视觉控制pid
    //PID_init(&init->vision_point->x_err_pid,PID_POSITION,x_err_k,);
    //PID_init(&init->vision_point->y_err_pid,PID_POSITION,y_err_k,);

    //电机清除
    DM_Clear(init->can_tx_data);
    can_tx_data(&hfdcan1,0x201,gimbalControl.can_tx_data,8);
    osDelay(50);
    can_tx_data(&hfdcan2,0x202,gimbalControl.can_tx_data,8);
    //电机使能
    DM_Enable(init->can_tx_data);
    can_tx_data(&hfdcan1,0x201,gimbalControl.can_tx_data,8);
    osDelay(50);
    can_tx_data(&hfdcan2,0x202,gimbalControl.can_tx_data,8);
}

//编码值转弧度
static float motor_ecd_to_rad(uint16_t ecd, uint16_t zero_ecd)
{
    uint16_t relative_ecd = ecd - zero_ecd;

    if (relative_ecd >= HALF_ECD_RANGE)
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
    update->yawEuler.absolute_angle = update->imu_point->Yaw;//获取绝对角
    update->yawEuler.relative_angle = update->yawMotor.motor_measurement.pos;//获取相对角
    //pitch数据更新
    update->pitchEuler.absolute_angle = update->imu_point->Pitch;//获取绝对角
    update->pitchEuler.relative_angle = update->pitchMotor.motor_measurement.pos;//获取相对角
}

//模式切换保存数据
static void gimbal_update_save(gimbal_control_struct *save)
{
    if (save == NULL)
    {
        return;
    }
    if (save->yawEuler.last_motorMode != MOTOR_GYRO && save->yawEuler.motorMode != MOTOR_GYRO)
    {
        save->yawEuler.absolute_angle_set = save->yawEuler.absolute_angle;
    }
    else if (save->yawEuler.last_motorMode != MOTOR_ENCODER && save->yawEuler.motorMode != MOTOR_ENCODER)
    {
        save->yawEuler.relative_angle_set = save->yawEuler.relative_angle;
    }

    if (save->pitchEuler.last_motorMode != MOTOR_GYRO && save->pitchEuler.motorMode != MOTOR_GYRO)
    {
        save->pitchEuler.absolute_angle_set = save->pitchEuler.absolute_angle;
    }
    else if (save->pitchEuler.last_motorMode != MOTOR_ENCODER && save->pitchEuler.motorMode != MOTOR_ENCODER)
    {
        save->pitchEuler.relative_angle_set = save->pitchEuler.relative_angle;
    }
}
//控制设置
static void gimbal_control_set(gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return;
    }
    static float euler_yaw_add = 0,euler_pitch_add = 0;
    //云台行为控制设置
    gimbal_behaviour_control_set(&euler_yaw_add,&euler_pitch_add,control);
    switch (control->yawEuler.motorMode)
    {
        case MOTOR_INIT:
        {

            break;
        }
        case MOTOR_GYRO:
        {
            gimbal_motor_gyro_control_set(control,&control->yawEuler,euler_yaw_add);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control_set(control,&control->yawEuler,euler_yaw_add);
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
            gimbal_motor_gyro_control_set(control,&control->pitchEuler,euler_pitch_add);
            break;
        }
        case MOTOR_ENCODER:
        {
            gimbal_motor_encoder_control_set(control,&control->pitchEuler,euler_pitch_add);
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
    if (control == NULL)
    {
        return;
    }
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


//编码值
static void gimbal_motor_encoder_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    static float euler_yaw_set =0,euler_pitch_set = 0;
    if (motor_control == &gimbal_control->yawEuler)
    {
        euler_yaw_set = motor_control->relative_angle_set;
        motor_control->relative_angle_set = fmodf(euler_yaw_set + add,PI);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        euler_pitch_set = motor_control->relative_angle_set;
        motor_control->relative_angle_set = fmodf(euler_pitch_set + add,PI);
    }
}
//陀螺仪
static void gimbal_motor_gyro_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    static float euler_yaw_set =0,euler_pitch_set = 0;
    if (motor_control == &gimbal_control->yawEuler)
    {
        euler_yaw_set = motor_control->absolute_angle_set;
        motor_control->absolute_angle_set = fmodf(euler_yaw_set + add,PI);
        //motor_control->absolute_angle_set = fmax(motor_control->absolute_angle_set,YAW_ABS_MAX);

    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        euler_pitch_set = motor_control->absolute_angle_set;
        motor_control->absolute_angle_set = fmodf(euler_pitch_set + add,PI);
        motor_control->absolute_angle_set = Math_Constrain(&motor_control->absolute_angle_set,PITCH_ABS_MIN,PITCH_ABS_MAX);

    }
}


//电机编码值控制
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    if (motor_control == &gimbal_control->yawEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        gimbal_control->yawMotor.give_vel = motor_control->euler_pos_control.out;;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        gimbal_control->pitchMotor.give_vel = motor_control->euler_pos_control.out;
    }
}

//电机陀螺仪控制
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    if (motor_control == &gimbal_control->yawEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        gimbal_control->yawMotor.give_vel = motor_control->euler_pos_control.out;;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        gimbal_control->pitchMotor.give_vel = motor_control->euler_pos_control.out;
    }
}