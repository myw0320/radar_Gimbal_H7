#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "can_comm_task.h"
#include "detect_task.h"
#include "calibrate_task.h"
#include "cmsis_os.h"

/* 全局变量 --------------------------------------------------------------- */

gimbal_control_struct gimbalControl; // 云台控制句柄

/* 静态函数声明 ------------------------------------------------------------- */

static void gimbal_init(gimbal_control_struct *init);
static void gimbal_update(gimbal_control_struct *update);
static void gimbal_update_save(gimbal_control_struct *save);
static void gimbal_control_set(gimbal_control_struct *control);
static void gimbal_control(gimbal_control_struct *control);
static void gimbal_motor_init_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_encoder_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);
static void gimbal_motor_gyro_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control,float add);

static void gimbal_motor_stop_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
static void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);

/* 函数实现 --------------------------------------------------------------- */

/**
 * @brief  云台主任务，1kHz调度
 */
void Gimbal_Task(void const *pvParameters)
{
    // 等待IMU初始化完成
    while(INS.ins_flag==0)
    {
        osDelay(1);
    }

    gimbal_init(&gimbalControl);

    while(1)
    {
        gimbal_update(&gimbalControl);          // 更新传感数据
        gimbal_control_set(&gimbalControl);      // 计算控制设定值
        gimbal_update_save(&gimbalControl);      // 模式切换时保存当前值
        gimbal_control(&gimbalControl);          // PID计算输出
        // 发送视觉数据
        Vision_SendData(gimbalControl.vision_point,gimbalControl.yawEuler.absolute_angle,gimbalControl.pitchEuler.absolute_angle,0);
        osDelay(1);
    }
}

/**
 * @brief  云台CAN发送包组装，根据遥控器状态选择发送使能/控制帧
 */
void UserGimbal_AddTxPacket(void)
{
    // 遥控器断开或DBUS超时 → 电机去使能
    if ((!gimbalControl.rc_fsi6_point->rc.sw[0] && !gimbalControl.rc_fsi6_point->rc.sw[1]) || toe_is_error(DBUS_TOE))
    {
        DM_Disable(gimbalControl.yaw_can_tx_data);
        can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
        DM_Disable(gimbalControl.pitch_can_tx_data);
        can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);
    }
    else
    {
        // yaw: 已使能则发MIT控制帧，否则发使能帧
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

        // pitch: 已使能则发MIT控制帧，否则发使能帧
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

/**
 * @brief  云台初始化：绑定传感器指针、使能电机、配置PID及滤波器
 */
static void gimbal_init(gimbal_control_struct *init)
{
    // 绑定外部数据源指针
    init->imu_point = &INS;
    init->vision_point = &visionData;
    init->radar_point = &radarData;
    init->rc_dt7_point = &dt7Data;
    init->rc_fsi6_point = &fsi6Data;

    // 使能电机（先发使能帧）
    DM_Enable(init->yaw_can_tx_data);
    can_tx_data(&YAW_CAN,YAW_CAN_ID,gimbalControl.yaw_can_tx_data);
    osDelay(10);
    DM_Enable(init->pitch_can_tx_data);
    can_tx_data(&PITCH_CAN,PITCH_CAN_ID,gimbalControl.pitch_can_tx_data);

    // yaw轴: 电机初始化 + 位置PID初始化
    DM_Init(&init->yawMotor,DM4310,MIT,0x01);
    PID_Init(&init->yawEuler.euler_abs_pos_control, PIDF_POSITION, YAW_ABS_POS_P, YAW_ABS_POS_I, YAW_ABS_POS_D,
             YAW_ABS_POS_F, YAW_ABS_POS_MAX_IOUT, YAW_ABS_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0005f, 0.01f, 0.2f, 0.0f, 0.0f, PID_D_ERR);

    PID_Init(&init->yawEuler.euler_rel_pos_control, PIDF_POSITION, YAW_REL_POS_P, YAW_REL_POS_I, YAW_REL_POS_D,
             YAW_REL_POS_F, YAW_REL_POS_MAX_IOUT, YAW_REL_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0f, 0.001f, 0.5f,0.0f, 0.0f, PID_D_ERR);

    // pitch轴: 电机初始化 + 位置PID初始化
    DM_Init(&init->pitchMotor,DM4310,MIT,0x02);
    PID_Init(&init->pitchEuler.euler_abs_pos_control, PIDF_POSITION, PITCH_ABS_POS_P, PITCH_ABS_POS_I, PITCH_ABS_POS_D,
             PITCH_ABS_POS_F, PITCH_ABS_POS_MAX_IOUT, PITCH_ABS_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.0005f, 0.01f, 0.2f, 0.0f, 0.0f, PID_D_ERR);

    PID_Init(&init->pitchEuler.euler_rel_pos_control, PIDF_POSITION, PITCH_REL_POS_P, PITCH_REL_POS_I, PITCH_REL_POS_D,
             PITCH_REL_POS_F, PITCH_REL_POS_MAX_IOUT, PITCH_REL_POS_MAX_OUT,
             GIMBAL_PID_PERIOD, 0.000f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_ERR);

    // 扫描初始化时间戳
    init->gimbalScan.scan_begin_time =  HAL_GetTick() * 0.001f;
    gimbal_update(init);

    // yaw轴限幅及零点
    init->yawEuler.absolute_angle_max = YAW_ABS_MAX;
    init->yawEuler.absolute_angle_min = YAW_ABS_MIN;
    init->yawEuler.absolute_zero_angle = YAW_ABS_ZERO;
    init->yawEuler.absolute_angle_set = 0;
    init->yawEuler.relative_angle_max = YAW_REL_MAX;
    init->yawEuler.relative_angle_min = YAW_REL_MIN;
    init->yawEuler.relative_zero_angle = YAW_REL_ZERO;
    init->yawEuler.relative_angle_set = 0;

    // pitch轴限幅及零点
    init->pitchEuler.absolute_angle_max = PITCH_ABS_MAX;
    init->pitchEuler.absolute_angle_min = PITCH_ABS_MIN;
    init->pitchEuler.absolute_zero_angle = PITCH_ABS_ZERO;
    init->pitchEuler.absolute_angle_set = 0;
    init->pitchEuler.relative_angle_max = PITCH_REL_MAX;
    init->pitchEuler.relative_angle_min = PITCH_REL_MIN;
    init->pitchEuler.relative_zero_angle = PITCH_REL_ZERO;
    init->pitchEuler.relative_angle_set = 0;

    // 陷波滤波器初始化
    Filter_Init(&init->yawEuler.euler_filter,FILTER_NOTCH,0.001f,500,1.0f,0.0f,5);
    Filter_Init(&init->pitchEuler.euler_filter,FILTER_NOTCH,0.001f,500,1.0f,0.0f,5);
}

/**
 * @brief  编码值转弧度
 * @param  ecd        当前编码值
 * @param  offset_ecd 零点偏移编码值
 * @return 弧度值
 */
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

/**
 * @brief  更新云台传感数据（IMU角度、电机位置、速度）
 */
static void gimbal_update(gimbal_control_struct *update)
{
    static float relative_yaw_temp = 0,relative_pitch_temp = 0;

    // yaw轴数据更新
     update->yawEuler.absolute_angle = update->imu_point->Yaw;   // IMU绝对角
     update->yawEuler.relative_angle = update->yawMotor.motor_measurement.pos; // 电机相对角
     update->yawEuler.vel = update->yawMotor.motor_measurement.vel;

     // pitch轴数据更新
     update->pitchEuler.absolute_angle = update->imu_point->Pitch; // IMU绝对角
     update->pitchEuler.relative_angle = update->pitchMotor.motor_measurement.pos; // 电机相对角
     update->pitchEuler.vel = update->pitchMotor.motor_measurement.vel;
}

/**
 * @brief  模式切换时保存当前角度设定值，防止切换瞬间跳变
 */
static void gimbal_update_save(gimbal_control_struct *save)
{
    if (save == NULL)
    {
        return;
    }

    // 非陀螺仪模式时，锁存当前绝对角为设定值
    if (save->yawEuler.motorMode != MOTOR_GYRO)
    {
        save->yawEuler.absolute_angle_set = save->yawEuler.absolute_angle;
    }
    // 非编码器模式时，锁存当前相对角为设定值
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
}

/**
 * @brief  根据当前电机模式，调用对应控制设定函数
 */
static void gimbal_control_set(gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return;
    }
    static float euler_yaw_add = 0,euler_pitch_add = 0;

    // 从行为模块获取角度增量
    gimbal_behaviour_control_set(&euler_yaw_add,&euler_pitch_add,control);

    // yaw轴模式分发
    switch (control->yawEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            gimbal_motor_init_control_set(control,&control->yawEuler);
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
    }

    // pitch轴模式分发
    switch (control->pitchEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            gimbal_motor_init_control_set(control,&control->pitchEuler);
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
    }
}

/**
 * @brief  根据当前电机模式，执行对应PID控制计算
 */
static void gimbal_control(gimbal_control_struct *control)
{
    if (control == NULL)
    {
        return;
    }

    // 更新云台行为模式（遥控器开关解析）
    gimbal_motor_mode_update(control);

    // yaw轴模式分发
    switch (control->yawEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            gimbal_motor_encoder_control(control,&control->yawEuler);
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
    }

    // pitch轴模式分发
    switch (control->pitchEuler.motorMode)
    {
        case MOTOR_INIT:
        {
            gimbal_motor_encoder_control(control,&control->pitchEuler);
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
    }
}

/**
 * @brief  初始化模式：将相对角设定值归零
 */
static void gimbal_motor_init_control_set(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (gimbal_control == NULL || motor_control == NULL)
    {
        return;
    }

    if (motor_control == &gimbal_control->yawEuler)
    {
        motor_control->relative_angle_set = 0.0f;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        motor_control->relative_angle_set = 0.0f;
    }
}

/**
 * @brief  编码器（相对角）模式：根据行为增量更新相对角设定值
 * @param  add 角度增量，自动模式下为目标绝对角度
 */
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
        // 自动攻击/移动/扫描模式：add为目标绝对角度，直接解算差值
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
        // 手动模式：在现有设定值上累加增量，RC摇杆控制
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
        // pitch轴限幅
        motor_control->relative_angle_set = Math_Constrain(&motor_control->relative_angle_set,PITCH_REL_MIN,PITCH_REL_MAX);
    }
}

/**
 * @brief  陀螺仪（绝对角）模式：根据行为增量更新绝对角设定值
 * @param  add 角度增量，自动模式下为目标绝对角度
 */
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
        // 自动模式：add为目标绝对角度
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
        // 手动模式：在现有设定值上累加增量
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
        // pitch轴限幅
        motor_control->absolute_angle_set = Math_Constrain(&motor_control->absolute_angle_set,PITCH_ABS_MIN,PITCH_ABS_MAX);
    }
}

/**
 * @brief  停止模式：速度置零
 */
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

/**
 * @brief  速度控制：将速度设定值写入电机控制字
 */
static void gimbal_motor_vel_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control)
{
    if (motor_control == &gimbal_control->yawEuler)
    {
        gimbal_control->yawMotor.give_kd = 1.2f;
        gimbal_control->yawMotor.give_vel = motor_control->vel_set;
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        gimbal_control->pitchMotor.give_kd = 1.4f;
        gimbal_control->pitchMotor.give_vel = motor_control->vel_set;
    }
}

/**
 * @brief  编码器位置环PID：相对角 → 速度设定值
 */
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

        motor_control->vel_set = motor_control->euler_rel_pos_control.out;
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

/**
 * @brief  陀螺仪位置环PID：绝对角 → 速度设定值
 */
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

        motor_control->vel_set = motor_control->euler_abs_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->yawEuler);
    }
    else if (motor_control == &gimbal_control->pitchEuler)
    {
        motor_control->euler_abs_pos_control.ref = motor_control->absolute_angle_set;
        motor_control->euler_abs_pos_control.fdb = motor_control->absolute_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&motor_control->euler_abs_pos_control);

        motor_control->vel_set = motor_control->euler_abs_pos_control.out;
        gimbal_motor_vel_control(gimbal_control,&gimbal_control->pitchEuler);
    }
}