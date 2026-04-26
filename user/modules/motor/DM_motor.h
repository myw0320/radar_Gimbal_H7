#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H
#include "stdint.h"
#include "math.h"
#include "bsp_can.h"




//达妙电机初始化数据
//#define Angle_Max 1.0f
#define Angle_Limit 12.5f
#define Omega_Limit 25.0f
#define Torque_Limit 10.0f
#define Current_Limit 10.0f


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum
{
	DM3507,
	DM4310,
	DM8009,
}dm_motor_type_enum;

typedef enum
{
	Disable = 0,
	Enable,
}dm_status_enum;

typedef enum
{
	MIT,
	ANGLE_OMEGA,
	OMEGA,
}dm_mode_enum;

typedef struct __attribute__((packed))
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;//rad
	float vel;//rad/s
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}dm_motor_t;//单电机结构体

typedef struct __attribute__((packed))
{
	uint8_t motor_id;
	uint16_t encoder;
	uint16_t last_encoder;
	int16_t rpm;
	uint16_t torque_current;
	uint16_t temperature;

	float omega;
}dm1to4_motor_t;


typedef struct __attribute__((packed))
{
	dm_motor_t motor_measurement;//电机原始数据
	dm_mode_enum mode;
	uint16_t can_id;

	float give_pos;//-PI~PI
	float give_vel;//rad/s
	float give_torque;
	float give_kp;
	float give_kd;
	int16_t give_cmd_current;
}dm_control_t;//电机控制结构体

typedef struct __attribute__((packed))
{
	dm1to4_motor_t motor_measurement;//电机原始数据
	dm_mode_enum mode;
	uint16_t can_id;


}dm1to4_control_t;

void DM_Enable(uint8_t *tx_data);
void DM_Disable(uint8_t *tx_data);
void DM_SaveZero(uint8_t *tx_data);
void DM_Clear(uint8_t *tx_data);
void DM_Init(dm_control_t *init,dm_motor_type_enum type, dm_mode_enum mode, uint8_t canid);
void DM_GetRxPacket(dm_motor_t *motor,uint8_t *rx_data);
void DM1TO4_GetRxPacket(dm1to4_motor_t *motor,uint8_t *rx_data);
void DM_AddTxPacket(dm_control_t *motor, uint8_t *tx_data);
void DM1TO4_AddTxPacket(uint8_t *tx_data, int16_t current1, int16_t current2, int16_t current3, int16_t current4);



#endif
