#include "Dji_motor.h"


void DJI_Init(dji_control_struct *init,dji_motor_type_enum type,uint8_t id)
{
    init->motor_measurement.motor_type = type;
    init->motor_measurement.motor_id = id;
}

void DJI_GetRxPacket(dji_control_struct *motor,uint8_t *rx_data)
{
    motor->motor_measurement.last_encoder = motor->motor_measurement.last_encoder;
    motor->motor_measurement.encoder = (uint16_t)(rx_data[0]<<8|rx_data[1]);
    motor->motor_measurement.rpm = (uint16_t)(rx_data[2]<<8|rx_data[3]);
    motor->motor_measurement.torque_current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
    motor->motor_measurement.temperature = rx_data[6];
}

void DJI_RxPacketUpdate(dji_control_struct *control)
{
   // control->angle
}


//打包发送数据
HAL_StatusTypeDef DJI_AddTxPacket(uint8_t master_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t tx_data[8];
    tx_data[0] = current1 >> 8;
    tx_data[1] = current1;
    tx_data[2] = current2 >> 8;
    tx_data[3] = current2;
    tx_data[4] = current3 >> 8;
    tx_data[5] = current3;
    tx_data[6] = current4 >> 8;
    tx_data[7] = current4;
    return can_tx_data(&hfdcan1,master_id,tx_data,8);
}
