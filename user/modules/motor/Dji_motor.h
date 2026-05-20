#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include "stdint.h"
#include "bsp_can.h"
#include "motor.h"



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

    float give_vel;//rad/s
    int16_t give_cmd_current;//给定电流值

}dji_control_struct;


void DJI_Init(dji_control_struct *init,dji_motor_type_enum type,uint8_t id);
void DJI_GetRxPacket(dji_motor_struct *motor,uint8_t *rx_data);
void DJI_RxPacketUpdate(dji_motor_struct *motor);
void DJI_AddTxPacket(uint8_t *tx_data, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
#endif
