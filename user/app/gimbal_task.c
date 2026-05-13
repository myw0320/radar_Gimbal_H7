#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "can_comm_task.h"
#include "detect_task.h"
#include "cmsis_os.h"
//#include "cap_comm.h"

gimbal_control_struct gimbalControl;//云台控制

static void gimbal_init(gimbal_control_struct *init);
static void gimbal_update(gimbal_control_struct *update);
static void gimbal_update_save(gimbal_control_struct *save);
static void gimbal_control_set(gimbal_control_struct *control);
static void gimbal_control(gimbal_control_struct *control);
static void gimbal_motor_init_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
static void gimbal_motor_encoder_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
static void gimbal_motor_gyro_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
//static void gimbal_motor_init_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_stop_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);

//24.03mm
/******/
const float yaw_pos_k[3] = {YAW_POS_P,YAW_POS_I,YAW_POS_D};
const float pitch_pos_k[3] = {PITCH_POS_P,PITCH_POS_I,PITCH_POS_D};

// const float x_err_k[3] = {0.0008f,0.00000035f,0.001f};
// const float y_err_k[3] = {0.0008f,0.00000035f,0.001f};


void Gimbal_Task(void const *pvParameters)
{
    while(INS.ins_flag==0)
    {//等待加速度收敛
        osDelay(1);
    }
    //初始化
    gimbal_init(&gimbalControl);

    while(1)
    {
        gimbal_update(&gimbalControl);//更新数据
        gimbal_control_set(&gimbalControl);//设置
        gimbal_update_save(&gimbalControl);
        gimbal_control(&gimbalControl);//计算控制值

        Vision_SendData(&gimbalControl->INS);
        osDelay(1);
    }
}
void UserGimbal_AddTxPacket(void)
{
    if (!gimbalControl.rc_fsi6_point->rc.sw[0] && !gimbalControl.rc_fsi6_point->rc.sw[1])//((toe_is_error(DBUS_TOE)) || gimbalControl.rc_point->rc.sw[0] == 2 && gimbalControl.rc_point->rc.sw[1] == 2)
    {
        DM_Disable(gimbalControl.yaw_can_tx_data);
        can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
        DM_Disable(gimbalControl.pitch_can_tx_data);
        can_tx_data(&PITCH_CAN,0x202,gimbalControl.pitch_can_tx_data);
    }
    else
    {
        if (gimbalControl.yawMotor.motor_measurement.state == 1)
        {
            DM_AddTxPacket(&gimbalControl.yawMotor,gimbalControl.yaw_can_tx_data);
            can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
        }
        else
        {
            DM_Enable(gimbalControl.yaw_can_tx_data);
            can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
        }

        if (gimbalControl.pitchMotor.motor_measurement.state == 1)
        {
            DM_AddTxPacket(&gimbalControl.pitchMotor,gimbalControl.pitch_can_tx_data);
            can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
        }
        else
        {
            DM_Enable(gimbalControl.pitch_can_tx_data);
            can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
        }
    }
}

void gimbal_clear()
{

}

static void gimbal_init(gimbal_control_struct *init)
{
    //获取指针地址
    init->imu_point = &INS;
    init->vision_point = &visionData;
    //init->radar_point = &radarData;
    init->rc_dt7_point = &dt7Data;
    init->rc_fsi6_point = &fsi6Data;
    //yaw数据初始化
    DM_Init(&init->yawMotor,DM4310,MIT,0x01);//电机控制初始化

    init->yawEuler.absolute_angle_max = YAW_ABS_MAX;
    init->yawEuler.absolute_angle_min = YAW_ABS_MIN;
    init->yawEuler.absolute_zero_angle = YAW_ABS_ZERO;
    init->yawEuler.absolute_angle_set = 0.0f;
    init->yawEuler.relative_angle_max = YAW_REL_MAX;
    init->yawEuler.relative_angle_min = YAW_REL_MIN;
    init->yawEuler.relative_zero_angle = YAW_REL_ZERO;
    init->yawEuler.relative_angle_set = YAW_REL_ZERO;
    PID_init(&init->yawEuler.euler_pos_control,PID_POSITION,yaw_pos_k,YAW_POS_MAX_OUT,YAW_POS_MIN_OUT,YAW_POS_MAX_IOUT,YAW_POS_MIN_IOUT);
    PID_advanced_config(&init->yawEuler.euler_pos_control,1,0.001f,0);
    //pitch数据初始化
    DM_Init(&init->pitchMotor,DM4310,MIT,0x06);//电机控制初始化
    init->pitchEuler.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEuler.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEuler.absolute_zero_angle = PITCH_ABS_ZERO;
    init->pitchEuler.absolute_angle_set = 0.0f;
    init->pitchEuler.relative_angle_max = PITCH_REL_MAX;
    init->pitchEuler.relative_angle_min = PITCH_REL_MIN;
    init->pitchEuler.relative_zero_angle = PITCH_REL_ZERO;
    init->pitchEuler.relative_angle_set = PITCH_REL_ZERO;
    PID_init(&init->pitchEuler.euler_pos_control,PID_POSITION,pitch_pos_k,PITCH_POS_MAX_OUT,PITCH_POS_MIN_OUT,PITCH_POS_MAX_IOUT,PITCH_POS_MIN_IOUT);
    PID_advanced_config(&init->pitchEuler.euler_pos_control,1,0.001f,0);
    //初始化视觉控制pid
    // PID_init(&init->visionToGimbal.x_err_pid,PID_POSITION,x_err_k,X_ERR_MAX_OUT,X_ERR_MIN_OUT,X_ERR_MAX_IOUT,X_ERR_MIN_IOUT);
    // PID_advanced_config(&init->visionToGimbal.x_err_pid,1,10,0);
    // //PID_integral_config(&init->visionToGimbal.x_err_pid,0,0.96f,1);
    // PID_init(&init->visionToGimbal.y_err_pid,PID_POSITION,y_err_k,Y_ERR_MAX_OUT,Y_ERR_MIN_OUT,Y_ERR_MAX_IOUT,Y_ERR_MIN_IOUT);
    // PID_advanced_config(&init->visionToGimbal.y_err_pid,1,10,0);
    //PID_integral_config(&init->visionToGimbal.y_err_pid,0,0.96f,1);

    //初始化视觉LFT
    // first_order_filter_init(&init->visionToGimbal.x_LFT,0.001f,x_err_a);
    // first_order_filter_init(&init->visionToGimbal.y_LFT,0.001f,y_err_a);

    init->gimbalScan.scan_begin_time =  HAL_GetTick() * 0.001f;

    // //电机清除
    DM_Clear(init->yaw_can_tx_data);
    can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
    osDelay(10);
    DM_Clear(init->pitch_can_tx_data);
    can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
    // 电机使能
    DM_Enable(init->yaw_can_tx_data);
    can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
    osDelay(10);
    DM_Enable(init->pitch_can_tx_data);
    can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
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
    //对数据进行一阶低通
    // first_order_filter_cali(&update->visionToGimbal.x_LFT,update->vision_point->receive_packet.x);
    // first_order_filter_cali(&update->visionToGimbal.y_LFT,update->vision_point->receive_packet.y);
    //
    // update->visionToGimbal.now_x = update->visionToGimbal.x_LFT.out;
    // update->visionToGimbal.now_y = update->visionToGimbal.y_LFT.out;
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
            gimbal_motor_init_control_set(control,&control->yawEuler,euler_yaw_add);
            break;
        }
        case MOTOR_STOP:
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
            gimbal_motor_init_control_set(control,&control->pitchEuler,euler_pitch_add);
            break;
        }
        case MOTOR_STOP:
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
            //gimbal_motor_init_control(control,&control->yawEuler);
            break;
        }
        case MOTOR_STOP:
        {
            gimbal_motor_stop_control(control,&control->yawEuler);
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
            //gimbal_motor_init_control(control,&control->pitchEuler);
            break;
        }
        case MOTOR_STOP:
        {
            gimbal_motor_stop_control(control,&control->pitchEuler);
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

static void gimbal_motor_init_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    //位置式
    static float euler_yaw_set =0,euler_pitch_set = 0;
    if (motor_control == &gimbal_control->yawEuler)
    {

        //euler_yaw_set = ;//
        //motor_control->relative_angle_set = fmodf(euler_yaw_set + add,PI);
        //motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,YAW_REL_MIN,YAW_REL_MAX);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        euler_pitch_set = motor_control->relative_angle_set;
        //motor_control->relative_angle_set = fmodf(euler_pitch_set + add,PI);
        //motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,PITCH_REL_MIN,PITCH_REL_MAX);
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
        motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,YAW_REL_MIN,YAW_REL_MAX);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        euler_pitch_set = motor_control->relative_angle_set;
        motor_control->relative_angle_set = fmodf(euler_pitch_set + add,PI);
        motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,PITCH_REL_MIN,PITCH_REL_MAX);
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
    static float delta_yaw = 0,delta_pitch = 0;

    if (motor_control == &gimbal_control->yawEuler)
    {
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE)
        {
            euler_yaw_set = add;
            delta_yaw = fmod(euler_yaw_set - motor_control->absolute_angle,2.0f * PI);
            if (delta_yaw > PI) delta_yaw -= 2.0f * PI;
            else if (delta_yaw < -PI) delta_yaw += 2.0f * PI;
            motor_control->absolute_angle_set = motor_control->absolute_angle + delta_yaw;
        }
        else
        {

            euler_yaw_set = motor_control->absolute_angle_set;

            delta_yaw = fmod(euler_yaw_set + add - motor_control->absolute_angle,2.0f * PI);
            if (delta_yaw > PI) delta_yaw -= 2.0f * PI;
            else if (delta_yaw < -PI) delta_yaw += 2.0f * PI;
            motor_control->absolute_angle_set = motor_control->absolute_angle + delta_yaw;
        }
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE)
        {
            euler_pitch_set = add;
            delta_pitch = fmod(euler_pitch_set - motor_control->absolute_angle,2.0f * PI);
            if (delta_pitch > PI) delta_pitch -= 2.0f * PI;
            else if (delta_pitch < -PI) delta_pitch += 2.0f * PI;
            motor_control->absolute_angle_set = motor_control->absolute_angle + delta_pitch;
        }
        else
        {
            euler_pitch_set = motor_control->absolute_angle_set;
            delta_pitch = fmod(euler_pitch_set + add - motor_control->absolute_angle,2.0f * PI);
            if (delta_pitch > PI) delta_pitch -= 2.0f * PI;
            else if (delta_pitch < -PI) delta_pitch += 2.0f * PI;
            motor_control->absolute_angle_set = motor_control->absolute_angle + delta_pitch;
        }
        motor_control->absolute_angle_set = Math_Constrain(&motor_control->absolute_angle_set,PITCH_ABS_MIN,PITCH_ABS_MAX);
    }
}


static void gimbal_motor_stop_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (motor_control == &gimbal_control->yawEuler)
    {
        gimbal_control->yawMotor.give_vel = 0;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        gimbal_control->pitchMotor.give_vel = 0;
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
        PID_calc(&motor_control->euler_pos_control,motor_control->relative_angle,motor_control->relative_angle_set);
        gimbal_control->yawMotor.give_vel = motor_control->euler_pos_control.out;;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->relative_angle,motor_control->relative_angle_set);
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
        gimbal_control->yawMotor.give_kd = 1.0f;
        gimbal_control->yawMotor.give_vel = motor_control->euler_pos_control.out;

    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        PID_calc(&motor_control->euler_pos_control,motor_control->absolute_angle,motor_control->absolute_angle_set);
        gimbal_control->pitchMotor.give_kd = 1.0f;
        gimbal_control->pitchMotor.give_vel = motor_control->euler_pos_control.out;

    }
}