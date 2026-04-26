#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include "stdint.h"
#include "bsp_can.h"

// #define M3508_K0 2.49688994e-6f
// #define M3508_K1 1.253e-07
// #define M3508_K2 1.23e-07
// #define M3508_K3 4.081f
//
// #define GM6020_K0 0.8130f
// #define GM6020_K1 -0.0005f
// #define GM6020_K2 6.0021f
// #define GM6020_K3 1.3715f

#define ENCODER 8191.0f
#define ENCODER_HALF 4096.0f
//减速比
#define M3508_RATIO 15.756f
#define M2006_RATIO 36
// 电流到输出的转化系数
#define M3508_Current_To_Out (20.0f/16384.0f)
#define GM6020_Current_To_Out (3.0f/16384.0f)
#define M2006_Current_To_Out (10.0f/10000.0f)

#define M3508_RPM_TO_VECTOR  0.000415809748903494517209f
#define GM6020_RPM_TO_VECTOR 0.001746201886833
#define M2006_RMP_TO_VECTOR 0.00290888208665721596153948461415f

#define RPM_TO_OMEGA 0.104719753f
typedef enum
{
    M3508,
    GM6020,
    M2006,
}dji_motor_type_enum;

typedef struct __attribute__((packed))
{
    dji_motor_type_enum motor_type;
    uint8_t motor_id;
    uint16_t encoder;
    uint16_t last_encoder;
    int16_t rpm;
    uint16_t torque_current;
    uint16_t temperature;

    float omega;

    //输出轴
    int32_t diff_enc;//两次角度差
    float ratio_angle;//经过减速器输出轴的角度
    int64_t total_pulses;
    double motor_revolutions;
    double output_revolutions;
}dji_motor_struct;

typedef struct
{
    dji_motor_struct motor_measurement;//电机原始数据
    float now_power;

    float give_vel;//rad/s
    float give_cmd_power;
    int16_t give_cmd_current;//给定电流值

}dji_control_struct;


void DJI_Init(dji_control_struct *init,dji_motor_type_enum type,uint8_t id);
void DJI_GetRxPacket(dji_motor_struct *motor,uint8_t *rx_data);
void DJI_RxPacketUpdate(dji_motor_struct *motor);
void DJI_AddTxPacket(uint8_t *tx_data, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
#endif
