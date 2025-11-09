#ifndef RADAR_GIMBAL_H7_DJI_MOTOR_H
#define RADAR_GIMBAL_H7_DJI_MOTOR_H
#include "math.h"
#include "arm_math.h"

#define M3508_K0 2.49688994e-6f
#define M3508_K1 1.253e-07
#define M3508_K2 1.23e-07
#define M3508_K3 4.081f

#define GM6020_K0 0.8130f
#define GM6020_K1 -0.0005f
#define GM6020_K2 6.0021f
#define GM6020_K3 1.3715f

#define Encoder 8191
#define Encoder_Half 8191/2
//3508新减速比
#define Reduction_ratio 18
#define KT 0.01526f//N*m/A 0.3*187/3591力矩电流常数
// 电流到输出的转化系数
#define M3508_Current_To_Out (20.0f/16384.0f)
#define GM6020_Current_To_Out (3.0f/16384.0f)
#define M2006_Current_To_Out (10.0f/10000.0f)
//转换式
#define RpmToOmega(rpm) (rpm*(float)PI/30.0f)
#define OmegaToRpm(omega) (omega *30.0f /(float)PI)
#define Torque_To_Icmd(torque) (torque / KT * M3508_Current_To_Out)
#define Icmd_To_Torque(icmd) (icmd * KT /M3508_Current_To_Out)
#define Encoder_To_PI(ecd) ((ecd/4095.5f) * PI - PI)

#define M3508_RPM_TO_VECTOR  0.000415809748903494517209f
#define GM6020_RPM_TO_VECTOR 0.001746201886833
#define M2006_RMP_TO_VECTOR 0.00290888208665721596153948461415f
typedef enum
{
    M3508,
    GM6020,
    M2006,
}dji_motor_type_enum;
typedef struct
{
    dji_motor_type_enum motor_type;
    uint16_t encoder;
    uint16_t last_encoder;
    uint16_t rpm;
    uint16_t torque_current;
    uint16_t temperature;
}dji_motor_struct;

typedef struct
{
    float angle;
    float angle_pi;
    float omega;
    float velocity;

    float given_current;//给定电流值
}dji_control_struct;
void DJI_Init(dji_motor_struct *init,dji_motor_type_enum type);
void DJI_GetRxPacket(dji_motor_struct *dji,uint8_t *rx_data);
void DJI_RxPacketUpdate(dji_control_struct *control);
void DJI_AddTxPacket(int16_t current1,int16_t current2,int16_t current3,int16_t current4,uint8_t *tx_data);
#endif
