#include "can_comm_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "cap_comm.h"


HAL_StatusTypeDef can_tx_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *tx_data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;// CAN ID
    TxHeader.IdType =  FDCAN_STANDARD_ID ;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = len;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;      //CAN发送错误指示
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
    TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;           //经典CAN
    TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;//不储存发送事件
    TxHeader.MessageMarker = 0;//消息标记

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, tx_data);
}


cap_rx_data_t cap_rx_tset;
uint8_t rx1_data[8];
uint8_t rx2_data[8];
uint8_t rx3_data[8];
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
    {
        FDCAN_RxHeaderTypeDef RxHeader1;
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1, rx1_data);
        switch(RxHeader1.Identifier)
        {
            case 0x01:
            {
                DM_GetRxPacket(&gimbalControl.yawMotor.motor_measurement,rx1_data);
                detect_hook(GIMBAL_YAW_TOE);
                break;
            }
            case 0x210:
            {
                CAP_GetRxPacket(&cap_rx_tset,rx1_data);//超电
                break;
            }
        }
    }
    else if (hfdcan == &hfdcan2)
    {

        FDCAN_RxHeaderTypeDef RxHeader2;
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader2, rx2_data);
        switch(RxHeader2.Identifier)
        {
            case 0x06:
            {
                DM_GetRxPacket(&gimbalControl.pitchMotor.motor_measurement,rx2_data);
                detect_hook(GIMBAL_PITCH_TOE);
                break;
            }
        }
    }
    else if (hfdcan == &hfdcan3)
    {
        FDCAN_RxHeaderTypeDef RxHeader3;

    }
}
