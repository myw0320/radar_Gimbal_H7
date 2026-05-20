#include "DM_motor.h"


/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}


void DM_Enable(uint8_t *tx_data)
{
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFC;
}


void DM_Disable(uint8_t *tx_data)
{
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFD;
}
void DM_SaveZero(uint8_t *tx_data)
{
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFE;
}
void DM_Clear(uint8_t *tx_data)
{
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xFF;
	tx_data[3] = 0xFF;
	tx_data[4] = 0xFF;
	tx_data[5] = 0xFF;
	tx_data[6] = 0xFF;
	tx_data[7] = 0xFB;
}

void DM_Init(dm_control_t *init,dm_motor_type_enum type, dm_mode_enum mode, uint8_t canid)
{
	init->can_id = canid;//canid
	init->mode = mode;
}

void DM_GetRxPacket(dm_motor_t *motor,uint8_t *rx_data)
{
	motor->id = (rx_data[0])&0x0F;
	motor->state = (rx_data[0])>>4;
	motor->p_int=(rx_data[1]<<8)|rx_data[2];
	motor->v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->pos = uint_to_float(motor->p_int, P_MIN, P_MAX, 16); // (-12.0,12.0)
	motor->vel = uint_to_float(motor->v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	motor->tor = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->Tmos = (float)(rx_data[6]);
	motor->Tcoil = (float)(rx_data[7]);
}

void DM1TO4_GetRxPacket(dm1to4_motor_t *motor,uint8_t *rx_data)
{
	motor->last_encoder = motor->encoder;
	motor->encoder = (uint16_t)(rx_data[0]<<8|rx_data[1]);
	motor->rpm = (int16_t)(rx_data[2]<<8|rx_data[3]);
	motor->torque_current = (int16_t)(rx_data[4]<<8|rx_data[5]);
	motor->temperature = rx_data[6];
	motor->omega = (float)motor->rpm * RPM_TO_OMEGA * 0.001f;
}


void DM_AddTxPacket(dm_control_t *motor, uint8_t *tx_data)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint8_t *pbuf,*vbuf;
	pos_tmp = float_to_uint(motor->give_pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(motor->give_vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(motor->give_kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(motor->give_kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(motor->give_torque, T_MIN,  T_MAX,  12);
	switch(motor->mode)
	{
		case MIT://MIT模式
		{
			tx_data[0] = (pos_tmp >> 8);
			tx_data[1] = pos_tmp;
			tx_data[2] = (vel_tmp >> 4);
			tx_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
			tx_data[4] = kp_tmp;
			tx_data[5] = (kd_tmp >> 4);
			tx_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
			tx_data[7] = tor_tmp;
			break;
		}
		case ANGLE_OMEGA://位置速度模式
		{
			pbuf = (uint8_t*)&motor->give_pos;
			vbuf = (uint8_t*)&motor->give_vel;

			tx_data[0] = *pbuf;
			tx_data[1] = *(pbuf+1);
			tx_data[2] = *(pbuf+2);
			tx_data[3] = *(pbuf+3);
			tx_data[4] = *vbuf;
			tx_data[5] = *(vbuf+1);
			tx_data[6] = *(vbuf+2);
			tx_data[7] = *(vbuf+3);
			break;
		}
		case OMEGA:
		{
			vbuf =(uint8_t*)&motor->give_vel;
			tx_data[0] = *vbuf;
			tx_data[1] = *(vbuf+1);
			tx_data[2] = *(vbuf+2);
			tx_data[3] = *(vbuf+3);
			break;
		}
	}
}
void DM1TO4_AddTxPacket(uint8_t *tx_data, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
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