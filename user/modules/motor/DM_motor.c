#include "cmsis_os.h"
#include "DM_motor.h"
#include "bsp_can.h"
#include "fdcan.h"
#if DM4310_1TO4

void DM_Init(dm_control_struct *init,dm_motor_type_enum type,uint8_t id)
{
	init->motor_measurement.motor_type = type;
	init->motor_measurement.motor_id = id;
}

void DM_GetRxPacket(dm_control_struct *motor,uint8_t *rx_data)
{
	motor->motor_measurement.encoder = (rx_data[0]<<8|rx_data[1]);
	motor->motor_measurement.rpm = (uint16_t)(rx_data[2]<<8|rx_data[3])/100;
	motor->motor_measurement.torque_current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
	motor->motor_measurement.motor_temperature = rx_data[6];
	motor->motor_measurement.pcb_temperature = rx_data[7];
}
void DM_RxPacketUpdate(dm_control_struct *update)
{
	update->angle = EncoderToAngle(update->motor_measurement.encoder);
	update->angle_pi =
	update->omega = RpmToOmega(update->motor_measurement.rpm);
}
//打包控制数据
HAL_StatusTypeDef DM_AddTxPacket(uint8_t master_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
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
	return can_send_data(&hfdcan1,master_id,tx_data,8);
}
#else
uint8_t DM_Motor_Enable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

uint8_t DM_Motor_Disable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

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
void DM4310_Enable(void)
{

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
void DM4310_Disable(void)
{

}
/**
************************************************************************
* @brief:      	DM4310_Init: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
void DM4310_Init(dm_motor_struct *init,uint8_t id,dm_mode_enum mode)
{
	DM4310_Enable();
	init->id = id;//canid
	init->mode = mode;
}

void DM4310_GetRxPacket(dm_motor_struct *motor,uint8_t *rx_data)
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

void DM4310_AddTxPacket(dm_control_struct *control)
{
	uint8_t tx_data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint8_t *pbuf,*vbuf;
	pos_tmp = float_to_uint(control->give_pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(control->give_vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(control->give_kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(control->give_kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(control->give_torque, T_MIN,  T_MAX,  12);
	switch(control->motor_measurement->mode)
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
			pbuf = (uint8_t*)&control->give_pos;
			vbuf = (uint8_t*)&control->give_vel;

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
			//vbuf =(uint8_t*)&motor->give_vel;
			break;
		}
	}
	can_send_data(&hfdcan1,control->motor_measurement->id,tx_data,8);
}



#endif
