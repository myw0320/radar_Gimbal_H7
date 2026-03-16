#include "vt13.h"



uint8_t vt13_rx_buf[2][SBUS_VT_RX_LEN];//¿ØÊý¾Ý°ü
vt13_data_stuct vt13Data;

void vt13_init(void)
{
    USART_RxDMA_MultiBuffer_Init(&huart10,(uint32_t *)vt13_rx_buf[0], (uint32_t *)vt13_rx_buf[1], VT13_RX_LEN);
}

void Vt13_RxPacketUpdate(volatile const uint8_t *VT13_buf, vt13_data_stuct *rc_ctrl)
{

    if (VT13_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    if(VT13_buf[0] == 0xA9 && VT13_buf[1] == 0x53)
    {
        rc_ctrl->rc.ch[0] = (VT13_buf[2] | (VT13_buf[3] << 8)) & 0x07ff;        //!< Channel 0
        rc_ctrl->rc.ch[1] = ((VT13_buf[3] >> 3) | (VT13_buf[4] << 5)) & 0x07ff; //!< Channel 1
        rc_ctrl->rc.ch[2] = ((VT13_buf[4] >> 6) | (VT13_buf[5] << 2) |          //!< Channel 2
                                                 (VT13_buf[6] << 10)) & 0x07ff;
        rc_ctrl->rc.ch[3] = ((VT13_buf[6] >> 1) | (VT13_buf[7] << 7)) & 0x07ff; //!< Channel 3
        rc_ctrl->rc.mode_sw = ((VT13_buf[7] >> 4) & 0x0003);
        rc_ctrl->rc.stop = ((VT13_buf[7] >> 6) & 0x01);                 //stop //go home
        rc_ctrl->rc.left_button = ((VT13_buf[7] >> 7) & 0x01);//fn           //left_button
        rc_ctrl->rc.right_button = ((VT13_buf[8] >> 0) & 0x01);               //right_button
        rc_ctrl->rc.wheel = ((VT13_buf[8] >> 1) | (VT13_buf[9] << 7)) & 0x07FF;
        rc_ctrl->rc.shutter = (VT13_buf[9] >> 4) & 0x01;//???

        rc_ctrl->mouse.x = (VT13_buf[10] | (VT13_buf[11] << 8));
        rc_ctrl->mouse.y = (VT13_buf[12] | (VT13_buf[13] << 8));
        rc_ctrl->mouse.z = (VT13_buf[14] | (VT13_buf[15] << 8));

        rc_ctrl->mouse.press_l = (VT13_buf[16] >> 0) & 0x03;
        rc_ctrl->mouse.press_r = (VT13_buf[16] >> 2) & 0x03;
        rc_ctrl->mouse.middle = (VT13_buf[16] >> 4) & 0x03;

        rc_ctrl->key.v = (VT13_buf[17] | (VT13_buf[18] << 8));

        rc_ctrl->crc16 = (VT13_buf[19] | (VT13_buf[20] << 8));

        rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
        rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
        rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
        rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
        rc_ctrl->rc.wheel -= RC_CH_VALUE_OFFSET;
    }
}

// void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
// {
//     /* Current memory buffer used is Memory 0 */
//     if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
//     {
//
//         /* Disable DMA */
//         __HAL_DMA_DISABLE(huart->hdmarx);
//
//         /* Switch Memory 0 to Memory 1*/
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
//
//         /* Reset the receive count */
//         __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_VT_RX_LEN);
//
//         /* Juge whether size is equal to the length of the received data */
//         if(Size == SBUS_VT_RX_LEN)
//         {
//             //Radar_GetRxPacket(radar_rx_buf[0],&radarData.receive_packet);
//         }
//
//     }
//     /* Current memory buffer used is Memory 1 */
//     else
//     {
//         /* Disable DMA */
//         __HAL_DMA_DISABLE(huart->hdmarx);
//
//         /* Switch Memory 1 to Memory 0*/
//         ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
//
//         /* Reset the receive count */
//         __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_VT_RX_LEN);
//
//         if(Size == SBUS_VT_RX_LEN)
//         {
//             //Radar_GetRxPacket(radar_rx_buf[1],&radarData.receive_packet);
//         }
//     }
// }
//

