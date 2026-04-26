#ifndef _H7_PS2_H
#define _H7_PS2_H
#include "main.h"
#include "stdbool.h"

#define DI   HAL_GPIO_ReadPin(DI_GPIO_Port,DI_Pin)

#define DO_H  HAL_GPIO_WritePin(DO_GPIO_Port,DO_Pin,SET)
#define DO_L  HAL_GPIO_WritePin(DO_GPIO_Port,DO_Pin,RESET)

#define CS_H  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,SET)
#define CS_L  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,RESET)

#define CLK_H  HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,SET)
#define CLK_L  HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,RESET)


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2			9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13	//三角形
#define PSB_CIRCLE      14	//圆圈
#define PSB_CROSS       15	//叉叉
#define PSB_SQUARE      16	//方框

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8



void InitPS2(void);
uint8_t PS2_RedLight(void);//判断是否为红灯模式
void PS2_ReadData(void);
void PS2_Cmd(uint8_t CMD);		  //
uint8_t PS2_DataKey(void);		  //键值读取
uint8_t PS2_AnologData(uint8_t button); //得到一个摇杆的模拟量
void PS2_ClearData(void);	  //清除数据缓冲区
void PS2_Vibration(uint8_t motor1, uint8_t motor2);//振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF

void PS2_EnterConfing(void);	 //进入配置
void PS2_TurnOnAnalogMode(void); //发送模拟量
void PS2_VibrationMode(void);    //振动设置
void PS2_ExitConfing(void);	     //完成配置
void PS2_SetInit(void);		     //配置初始化


bool PS2_NewButtonState( uint16_t button );
bool PS2_Button( uint16_t button );
bool PS2_ButtonPressed( uint16_t button );
bool PS2_ButtonReleased( uint16_t button );

#endif //_H7_PS2_H
