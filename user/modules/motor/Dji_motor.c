#include "Dji_motor.h"



/**
 * @brief 初始化电机控制结构体
 * 
 * @param init 电机控制结构体
 * @param type 电机类型
 * @param id 电机ID
 */
void DJI_Init(dji_control_struct *init, dji_motor_type_enum type, uint8_t id)
{
    init->motor_measurement.motor_type = type;
    init->motor_measurement.motor_id = id;
}

/**
 * @brief 从接收数据中获取电机测量数据
 *
 * @param motor 电机控制结构体
 * @param rx_data 接收的数据
 */
void DJI_GetRxPacket(dji_motor_struct *motor,uint8_t *rx_data)
{
    int16_t tmp_rpm;
    motor->last_encoder = motor->encoder;
    motor->encoder = (uint16_t)((rx_data[0]<<8)|rx_data[1]);
    tmp_rpm = (int16_t)((rx_data[2]<<8)|rx_data[3]);
    motor->rpm = (float)tmp_rpm;
    motor->torque_current = (uint16_t)((rx_data[4]<<8)|rx_data[5]);
    motor->temperature = rx_data[6];
    //直接转换数据
    motor->omega = (float)motor->rpm * RPM_TO_OMEGA;

    if (motor->motor_type == M2006 || motor->motor_type == M3508)
    {
        motor->diff_enc = motor->encoder - motor->last_encoder;
        if (motor->diff_enc > ENCODER_HALF)
        {
            motor->diff_enc -= ENCODER;
        }
        else if (motor->diff_enc < -ENCODER_HALF)
        {
            motor->diff_enc += ENCODER;
        }
        motor->total_pulses += motor->diff_enc;
        motor->motor_revolutions = (double)motor->total_pulses / ENCODER;
        if (motor->motor_type == M2006)
        {
            motor->output_revolutions = motor->motor_revolutions / M2006_RATIO;
        }
        else if (motor->motor_type == M3508)
        {
            motor->output_revolutions = motor->motor_revolutions / M3508_RATIO;
        }

        motor->ratio_angle = (float)(fmod(motor->output_revolutions * 360.0f,360.0f));
        if (motor->ratio_angle < 0) motor->ratio_angle += 360.0f;
    }
}


/**
 * @brief 根据接收到的数据包更新角度和角速度值
 * @param update 指向电机控制结构体的指针
 */
void DJI_RxPacketUpdate(dji_motor_struct *motor)
{
    // control->angle
}

/**
 * @brief 添加发送数据包
 * 
 * @param tx_data 发送的数据
 * @param current1 电流1
 * @param current2 电流2
 * @param current3 电流3
 * @param current4 电流4
 */
void DJI_AddTxPacket(uint8_t *tx_data, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    tx_data[0] = current1 >> 8;
    tx_data[1] = current1;
    tx_data[2] = current2 >> 8;
    tx_data[3] = current2;
    tx_data[4] = current3 >> 8;
    tx_data[5] = current3;
    tx_data[6] = current4 >> 8;
    tx_data[7] = current4;
}
