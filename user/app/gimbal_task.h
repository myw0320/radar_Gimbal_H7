
#ifndef RADAR_GIMBAL_H7_GIMBAL_TASK_H
#define RADAR_GIMBAL_H7_GIMBAL_TASK_H
#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "ins_task.h"
#include "vision_task.h"
#include "radar_task.h"
#include "user_lib.h"
#include "fsi6.h"
#include "dt7.h"
#include "Dji_motor.h"
#include "DM_motor.h"
#include "pid.h"
#include "Mathh.h"
#include "cap_comm.h"
#include "PID_control.h"

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)


#define ECD_RANGE 8191
#define HALF_ECD_RANGE 4096
#define MOTOR_ECD_TO_RAD 0.000766990394f //����ֵת��Ϊ����ֵ  2*PI/8192
//yaw���������ֵ
#define YAW_ZERO_ENCODER 4096
//pitch���������ֵ
#define PITCH_ZERO_ENCODER 4096
//yaw�������޷�ֵ
#define YAW_ABS_ZERO 0.0f
#define YAW_ABS_MAX 0.6f
#define YAW_ABS_MIN -0.6f
#define YAW_REL_ZERO 0.0f
#define YAW_REL_MAX 3.14f
#define YAW_REL_MIN -3.14f
//pitch�������޷�ֵ
#define PITCH_ABS_ZERO 0.0f
#define PITCH_ABS_MAX 0.36f
#define PITCH_ABS_MIN -0.43f
#define PITCH_REL_ZERO 0.0f
#define PITCH_REL_MAX 3.14f
#define PITCH_REL_MIN -3.14f


#define GIMBAL_PID_PERIOD 0.001f

#define YAW_ABS_POS_P 60.0f
#define YAW_ABS_POS_I 15.0f
#define YAW_ABS_POS_D 10.0f
#define YAW_ABS_POS_F 0.1f
#define YAW_ABS_POS_MAX_OUT 10.0f
#define YAW_ABS_POS_MIN_OUT -10.0f
#define YAW_ABS_POS_MAX_IOUT 1.0f
#define YAW_ABS_POS_MIN_IOUT -1.0f

#define YAW_REL_POS_P 15.0f
#define YAW_REL_POS_I 0.0f
#define YAW_REL_POS_D 2.0f
#define YAW_REL_POS_F 0.0f
#define YAW_REL_POS_MAX_OUT 2.0f
#define YAW_REL_POS_MIN_OUT -2.0f
#define YAW_REL_POS_MAX_IOUT 1.0f
#define YAW_REL_POS_MIN_IOUT -1.0f

#define YAW_VEL_P 0.80f
#define YAW_VEL_I 0.0f
#define YAW_VEL_D 0.003f
#define YAW_VEL_F 0.0f
#define YAW_VEL_MAX_OUT 3.0f
#define YAW_VEL_MIN_OUT -3.0f
#define YAW_VEL_MAX_IOUT 1.0f
#define YAW_VEL_MIN_IOUT -1.0f


#define PITCH_ABS_POS_P 85.0f
#define PITCH_ABS_POS_I 30.0f
#define PITCH_ABS_POS_D 20.0f
#define PITCH_ABS_POS_F 2.0f
#define PITCH_ABS_POS_MAX_OUT 10.0f
#define PITCH_ABS_POS_MIN_OUT -10.0f
#define PITCH_ABS_POS_MAX_IOUT 2.0f
#define PITCH_ABS_POS_MIN_IOUT -2.0f

#define PITCH_REL_POS_P 15.0f
#define PITCH_REL_POS_I 0.025f
#define PITCH_REL_POS_D 0.03f
#define PITCH_REL_POS_F 0.0f
#define PITCH_REL_POS_MAX_OUT 5.0
#define PITCH_REL_POS_MIN_OUT -5.0
#define PITCH_REL_POS_MAX_IOUT 1.0
#define PITCH_REL_POS_MIN_IOUT -1.0

#define PITCH_VEL_P 1.0f
#define PITCH_VEL_I 0.0f
#define PITCH_VEL_D 0.0003f
#define PITCH_VEL_F 0.0f
#define PITCH_VEL_MAX_OUT 3.0
#define PITCH_VEL_MIN_OUT -3.0
#define PITCH_VEL_MAX_IOUT 1.0
#define PITCH_VEL_MIN_IOUT -1.0


typedef enum
{
    MOTOR_INIT = 0,
    MOTOR_GYRO,
    MOTOR_ENCODER,
    MOTOR_STOP
}motor_mode_enum;
//����ŷ���ǽṹ��
typedef struct
{
    motor_mode_enum motorMode;
    motor_mode_enum last_motorMode;

    pid_ctrl_t euler_abs_pos_control;//角度环
    pid_ctrl_t euler_rel_pos_control;//角速度环
    pid_ctrl_t euler_vel_control;

    float vel_set;
    float vel;

    float absolute_zero_angle;
    float absolute_angle;
    float absolute_angle_set;
    float absolute_angle_max;
    float absolute_angle_min;

    float relative_zero_angle;
    float relative_angle;
    float relative_angle_set;
    float relative_angle_max;
    float relative_angle_min;
}gimbal_motor_t;

typedef struct
{
    // ɨ���ͨ�˲��ṹ��
    first_order_filter_type_t pitch_auto_scan_first_order_filter;
    first_order_filter_type_t yaw_auto_scan_first_order_filter;

    //yaw������ֵ
    float yaw_center_value;
    //pitch������ֵ
    float pitch_center_value;

    //yaw���˶�����
    float yaw_range;
    //pitch���˶�����
    float pitch_range;

    //��ǰ����ʱ�� ��λs
    float scan_run_time;
    //��ʼ��ʱʱ�� ��λs
    float scan_begin_time;

    //yaw��ɨ������ ��λs
    float scan_yaw_period;
    //pitch��ɨ������  ��λs
    float scan_pitch_period;

    // �Զ�ɨ�����ø���ֵ
    float auto_scan_AC_set_yaw;
    float auto_scan_AC_set_pitch;

    int8_t last_yaw_dir;
    float pitch_accumulated; // ���ڼ�¼����ɨ���� Pitch �ᵱǰ�ߵ�������
}scan_struct;




typedef struct
{
    const INS_t *imu_point;//����������
    const dt7_data_struct *rc_dt7_point;//ң��������
    const fsi6_data_struct *rc_fsi6_point;

    radar_data_t *radar_point;
    vision_data_t *vision_point;//�Ӿ�����


    gimbal_motor_t yawEuler;
    dm_control_t yawMotor;//yaw����ṹ��
    gimbal_motor_t pitchEuler;
    dm_control_t pitchMotor;//pitch����ṹ��

    scan_struct gimbalScan;//�Զ�ɨ��
    bool enable;

    float current_time;

    uint8_t yaw_can_tx_data[8];
    uint8_t pitch_can_tx_data[8];
}gimbal_control_struct;

extern gimbal_control_struct gimbalControl;//��̨����
void UserGimbal_AddTxPacket(void);
void gimbal_motor_encoder_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
void gimbal_motor_gyro_control(gimbal_control_struct *gimbal_control,gimbal_motor_t *motor_control);
#endif //RADAR_GIMBAL_H7_GIMBAL_TASK_H