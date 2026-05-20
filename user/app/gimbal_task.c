#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "can_comm_task.h"
#include "detect_task.h"
#include "calibrate_task.h"
#include "cmsis_os.h"
//#include "cap_comm.h"

gimbal_control_struct gimbalControl;//��̨����
extern cali_flag_t cali_flag;
extern float gimbal_yaw_center_offset;

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
// void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
// void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);

//24.03mm



void Gimbal_Task(void const *pvParameters)
{
    while(INS.ins_flag==0)
    {//�ȴ����ٶ�����
        osDelay(1);
    }
    //��ʼ��
    gimbal_init(&gimbalControl);
    while(1)
    {
        gimbal_update(&gimbalControl);//��������
        gimbal_control_set(&gimbalControl);//����
        gimbal_update_save(&gimbalControl);
        gimbal_control(&gimbalControl);//�������ֵ
        //�����Ӿ�����
        Vision_SendData(gimbalControl.vision_point,gimbalControl.yawEuler.absolute_angle,gimbalControl.pitchEuler.absolute_angle,0);
        osDelay(1);
    }
}
void UserGimbal_AddTxPacket(void)
{
    if ((!gimbalControl.rc_fsi6_point->rc.sw[0] && !gimbalControl.rc_fsi6_point->rc.sw[1]) || toe_is_error(DBUS_TOE))// || gimbalControl.rc_point->rc.sw[0] == 2 && gimbalControl.rc_point->rc.sw[1] == 2)
    {
        DJI_AddTxPacket(gimbalControl.yaw_can_tx_data,0,0,0,0);
        can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);

        DM1TO4_AddTxPacket(gimbalControl.pitch_can_tx_data,0,0,0,0);
        can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
        // DM_Disable(gimbalControl.yaw_can_tx_data);
        // can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
        // DM_Disable(gimbalControl.pitch_can_tx_data);
        // can_tx_data(&PITCH_CAN,0x202,gimbalControl.pitch_can_tx_data);
    }
    else
    {
        DJI_AddTxPacket(gimbalControl.yaw_can_tx_data,gimbalControl.yawMotor.give_cmd_current,0,0,0);
        can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);

        DM1TO4_AddTxPacket(gimbalControl.pitch_can_tx_data,0,gimbalControl.pitchMotor.give_cmd_current,0,0);
        can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);

    }
}

void gimbal_clear()
{

}

static void gimbal_init(gimbal_control_struct *init)
{
    //��ȡָ���ַ
    init->imu_point = &INS;
    init->vision_point = &visionData;
    init->radar_point = &radarData;
    init->rc_dt7_point = &dt7Data;
    init->rc_fsi6_point = &fsi6Data;


    //yaw���ݳ�ʼ��
    //DM_Init(&init->yawMotor,DM4310,MIT,0x01);//������Ƴ�ʼ��
    DJI_Init(&init->yawMotor,GM6020,0x01);
    PID_Init(&init->yawEuler.euler_abs_pos_control, YAW_ABS_POS_P, YAW_ABS_POS_I, YAW_ABS_POS_D,
             YAW_ABS_POS_F, YAW_ABS_POS_MAX_IOUT, YAW_ABS_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_ERR);

    PID_Init(&init->yawEuler.euler_rel_pos_control, YAW_REL_POS_P, YAW_REL_POS_I, YAW_REL_POS_D,
             YAW_REL_POS_F, YAW_REL_POS_MAX_IOUT, YAW_REL_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_ERR);

    PID_Init(&init->yawEuler.euler_vel_control,YAW_VEL_P,YAW_VEL_I,YAW_VEL_D,
            YAW_VEL_F,YAW_VEL_MAX_IOUT,YAW_VEL_MAX_OUT,
            GIMBAL_PID_PERIOD,0.01f,0.0f,0.0f,0.0f,PID_D_NOW);

    first_order_filter_init(&init->yawEuler.euler_filter,0.001f,0.45f);
    first_order_filter_init(&init->pitchEuler.euler_filter,0.001f,0.45f);
    //pitch
    //DM_Init(&init->pitchMotor,DM4310,MIT,0x06);//

    PID_Init(&init->pitchEuler.euler_abs_pos_control, PITCH_ABS_POS_P, PITCH_ABS_POS_I, PITCH_ABS_POS_D,
             PITCH_ABS_POS_F, PITCH_ABS_POS_MAX_IOUT, PITCH_ABS_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0005f, 0.01f, 0.2f, 0.0f, PID_D_ERR);

    PID_Init(&init->pitchEuler.euler_rel_pos_control, PITCH_REL_POS_P, PITCH_REL_POS_I, PITCH_REL_POS_D,
             PITCH_REL_POS_F, PITCH_REL_POS_MAX_IOUT, PITCH_REL_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.01f, 0.0f, 0.0f, 0.0f, PID_D_ERR);

    PID_Init(&init->pitchEuler.euler_vel_control,PITCH_VEL_P,PITCH_VEL_I,PITCH_VEL_D,
            PITCH_VEL_F,PITCH_VEL_MAX_IOUT,PITCH_VEL_MAX_OUT,
            GIMBAL_PID_PERIOD,0.0f,0.0f,0.0f,0.0f,PID_D_NOW);

    init->gimbalScan.scan_begin_time =  HAL_GetTick() * 0.001f;
    gimbal_update(init);
    init->yawEuler.absolute_angle_max = YAW_ABS_MAX;
    init->yawEuler.absolute_angle_min = YAW_ABS_MIN;
    init->yawEuler.absolute_zero_angle = YAW_ABS_ZERO;
    init->yawEuler.absolute_angle_set = 0;
    init->yawEuler.relative_angle_max = YAW_REL_MAX;
    init->yawEuler.relative_angle_min = YAW_REL_MIN;
    init->yawEuler.relative_zero_angle = YAW_REL_ZERO;
    init->yawEuler.relative_angle_set = 0;

    init->pitchEuler.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEuler.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEuler.absolute_zero_angle = PITCH_ABS_ZERO;
    init->pitchEuler.absolute_angle_set = 0;
    init->pitchEuler.relative_angle_max = PITCH_REL_MAX;
    init->pitchEuler.relative_angle_min = PITCH_REL_MIN;
    init->pitchEuler.relative_zero_angle = PITCH_REL_ZERO;
    init->pitchEuler.relative_angle_set = 0;
}


//编码值转弧度
static float motor_ecd_to_rad(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
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

//���ݸ���
static void gimbal_update(gimbal_control_struct *update)
{
    static float relative_yaw_temp = 0,relative_pitch_temp = 0;

    //yaw���ݸ���
     update->yawEuler.absolute_angle = update->imu_point->Yaw;//��ȡ���Խ�
     update->yawEuler.relative_angle = motor_ecd_to_rad(update->yawMotor.motor_measurement.encoder,0);//��ȡ��Խ�
     update->yawEuler.vel = first_order_filter_cali(&update->yawEuler.euler_filter,update->yawMotor.motor_measurement.omega);
     //pitch���ݸ���
     update->pitchEuler.absolute_angle = update->imu_point->Pitch;//��ȡ���Խ�
     update->pitchEuler.relative_angle = motor_ecd_to_rad(update->pitchMotor.motor_measurement.encoder,0);
     update->pitchEuler.vel = first_order_filter_cali(&update->pitchEuler.euler_filter,update->yawMotor.motor_measurement.omega);

}

//ģʽ�л���������
static void gimbal_update_save(gimbal_control_struct *save)
{
    if (save == NULL)
    {
        return;
    }

    if (save->yawEuler.motorMode != MOTOR_GYRO)
    {
        save->yawEuler.absolute_angle_set = save->yawEuler.absolute_angle;
    }
    if (save->yawEuler.motorMode != MOTOR_ENCODER)
    {
        save->yawEuler.relative_angle_set = save->yawEuler.relative_angle;
    }

    if (save->pitchEuler.motorMode != MOTOR_GYRO)
    {
        save->pitchEuler.absolute_angle_set = save->pitchEuler.absolute_angle;
    }
    if (save->pitchEuler.motorMode != MOTOR_ENCODER)
    {
        save->pitchEuler.relative_angle_set = save->pitchEuler.relative_angle;
    }

    // if (save->yawEuler.last_motorMode != save->yawEuler.motorMode)
    // {
    //     if (save->yawEuler.motorMode == MOTOR_GYRO)
    //     {
    //         PID_Clear(&save->yawEuler.euler_abs_pos_control);
    //     }
    //     else if (save->yawEuler.motorMode == MOTOR_ENCODER)
    //     {
    //         PID_Clear(&save->yawEuler.euler_rel_pos_control);
    //     }
    // }

    // if (save->pitchEuler.last_motorMode != save->pitchEuler.motorMode)
    // {
    //     if (save->pitchEuler.motorMode == MOTOR_GYRO)
    //     {
    //         PID_Clear(&save->pitchEuler.euler_abs_pos_control);
    //     }
    //     else if (save->pitchEuler.motorMode == MOTOR_ENCODER)
    //     {
    //         PID_Clear(&save->pitchEuler.euler_rel_pos_control);
    //     }
    // }
}
//��������
static void gimbal_control_set(gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return;
    }
    static float euler_yaw_add = 0,euler_pitch_add = 0;
    //��̨��Ϊ��������
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
    //������̨�������Ϊ
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
    //λ��ʽ
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
//����ֵ
static void gimbal_motor_encoder_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }

    static float euler_yaw_set =0,euler_pitch_set = 0;
    static float delta_yaw = 0,delta_pitch = 0;
    if (motor_control == &gimbal_control->yawEuler)
    {
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE ||
            gimbalMode == GIMBAL_MANUAL_ATTACK_MODE ||
            gimbalMode == GIMBAL_AUTO_MOVE_MODE ||
            gimbalMode == GIMBAL_AUTO_SCAN_MODE)
        {
            euler_yaw_set = add;
            delta_yaw = fmod(euler_yaw_set - motor_control->relative_angle,2.0f * PI);
            if (delta_yaw > PI) delta_yaw -= 2.0f * PI;
            else if (delta_yaw < -PI) delta_yaw += 2.0f * PI;
            motor_control->relative_angle_set = motor_control->relative_angle + delta_yaw;
        }
        else
        {
            euler_yaw_set = motor_control->relative_angle_set;
            delta_yaw = fmod(euler_yaw_set + add - motor_control->relative_angle,2.0f * PI);
            if (delta_yaw > PI) delta_yaw -= 2.0f * PI;
            else if (delta_yaw < -PI) delta_yaw += 2.0f * PI;
            motor_control->relative_angle_set = motor_control->relative_angle + delta_yaw;
        }
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE ||
            gimbalMode == GIMBAL_MANUAL_ATTACK_MODE ||
            gimbalMode == GIMBAL_AUTO_MOVE_MODE ||
            gimbalMode == GIMBAL_AUTO_SCAN_MODE)
        {
            euler_pitch_set = add;
            delta_pitch = fmod(euler_pitch_set - motor_control->relative_angle,2.0f * PI);
            if (delta_pitch > PI) delta_pitch -= 2.0f * PI;
            else if (delta_pitch < -PI) delta_pitch += 2.0f * PI;
            motor_control->relative_angle_set = motor_control->relative_angle + delta_pitch;
        }
        else
        {
            euler_pitch_set = motor_control->relative_angle_set;
            delta_pitch = fmod(euler_pitch_set + add - motor_control->relative_angle,2.0f * PI);
            if (delta_pitch > PI) delta_pitch -= 2.0f * PI;
            else if (delta_pitch < -PI) delta_pitch += 2.0f * PI;
            motor_control->relative_angle_set = motor_control->relative_angle + delta_pitch;
        }
        motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,PITCH_REL_MIN,PITCH_REL_MAX);
    }
}


//������
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
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE ||
            gimbalMode == GIMBAL_MANUAL_ATTACK_MODE ||
            gimbalMode == GIMBAL_AUTO_MOVE_MODE ||
            gimbalMode == GIMBAL_AUTO_SCAN_MODE)
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
        if (gimbalMode == GIMBAL_AUTO_ATTACK_MODE ||
            gimbalMode == GIMBAL_MANUAL_ATTACK_MODE ||
            gimbalMode == GIMBAL_AUTO_MOVE_MODE ||
            gimbalMode == GIMBAL_AUTO_SCAN_MODE)
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

static void gimbal_motor_vel_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (motor_control == &gimbal_control->yawEuler)
    {
         motor_control->euler_vel_control.ref = motor_control->vel_set;
         motor_control->euler_vel_control.fdb = motor_control->vel;
         PID_Calc(&motor_control->euler_vel_control);


        gimbal_control->yawMotor.give_cmd_current = (int16_t)motor_control->euler_vel_control.out;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        motor_control->euler_vel_control.ref = motor_control->vel_set;
        motor_control->euler_vel_control.fdb = motor_control->vel;
        PID_Calc(&motor_control->euler_vel_control);


        gimbal_control->pitchMotor.give_cmd_current = (int16_t)motor_control->euler_vel_control.out;
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

void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }
    if (motor_control == &gimbal_control->yawEuler)
    {
        motor_control->euler_rel_pos_control.ref = motor_control->relative_angle_set;
        motor_control->euler_rel_pos_control.fdb = motor_control->relative_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor_control->euler_rel_pos_control);

        //motor_control->vel_set = motor_control->euler_rel_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->yawEuler);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        motor_control->euler_rel_pos_control.ref = motor_control->relative_angle_set;
        motor_control->euler_rel_pos_control.fdb = motor_control->relative_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor_control->euler_rel_pos_control);

        motor_control->vel_set = motor_control->euler_rel_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->pitchEuler);
    }
}

void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }

    if (motor_control == &gimbal_control->yawEuler)
    {
        motor_control->euler_abs_pos_control.ref = motor_control->absolute_angle_set;
        motor_control->euler_abs_pos_control.fdb = motor_control->absolute_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor_control->euler_abs_pos_control);

        //motor_control->vel_set = motor_control->euler_abs_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->yawEuler);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        motor_control->euler_abs_pos_control.ref = motor_control->absolute_angle_set;
        motor_control->euler_abs_pos_control.fdb = motor_control->absolute_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor_control->euler_abs_pos_control);

        //motor_control->vel_set = motor_control->euler_abs_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->pitchEuler);
    }
}

