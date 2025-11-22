#ifndef RADAR_GIMBAL_H7_DM_MOTOR_H
#define RADAR_GIMBAL_H7_DM_MOTOR_H
#include "stdint.h"
#include "math.h"
#include "bsp_can.h"


#define DM4310_1TO4 1

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

#define RpmToOmega(rpm) (rpm*(float)M_PI/30.0f)
#define EncoderToAngle(encoder) (encoder*0.043950f - 180.0f)
#if DM4310_1TO4
typedef enum
{
	DM4010,
	DM4310,
	DM8009,
}dm_motor_type_enum;
typedef struct __attribute__((packed))
{
	dm_motor_type_enum motor_type;
	uint8_t motor_canid;
	uint16_t encoder;//编码值
	uint16_t rpm;//rpm值
	uint16_t torque_current;//扭矩电流
	uint8_t motor_temperature;
	uint8_t pcb_temperature;
}dm_motor_struct;//电机原始数据

typedef struct
{
	dm_motor_struct motor_measurement;//电机原始数据
	float angle;
	float angle_pi;
	float omega;
	float velocity;

	int16_t give_cmd_current;//给定电流值
}dm_control_struct;

void DM_Init(dm_control_struct *init,dm_motor_type_enum type,uint8_t id);
void DM_GetRxPacket(dm_control_struct *motor,uint8_t *rx_data);
void DM_RxPacketUpdate(dm_control_struct *update);
HAL_StatusTypeDef DM_AddTxPacket(uint8_t master_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
#else
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

typedef struct
{
	dm_mode_enum mode;
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}dm_motor_struct;//单电机结构体

typedef struct
{
	dm_motor_struct *motor_measurement;//电机原始数据

	float give_pos;//-PI~PI
	float give_vel;
	float give_torque;
	float give_kp;
	float give_kd;
	int16_t give_cmd_current;
}dm_control_struct;//电机控制结构体

#endif



#endif
