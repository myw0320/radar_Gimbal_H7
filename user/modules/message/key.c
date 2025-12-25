#include "key.h"
#include "tim.h"
#include "string.h"

/*********按键*********/
key_struct keyMessage;//

//按键状态机更新
void Key_StatusUpdate(key_struct *key)
{
	key->key = K1;
	// switch (key->keyStatus)
	// {
	// 	case KEY_IDLE:
	// 	{
	// 		key->keyNum = 0;
	// 		if (K1)
	// 		{
	// 			key->keyStatus = KEY_SINGLE;
	// 			key->startTime = HAL_GetTick();//获取起始时间
	// 		}
	// 		break;
	// 	}
	// 	case KEY_SINGLE:
	// 	{
	// 		key->keyNum++;
	// 		if (KEY1)
	// 		{
	// 			key->currentTime = HAL_GetTick();
	// 			if (key->currentTime - key->startTime >= 500)
	// 			{
	// 				key->keyStatus = KEY_PLAYING;//切换为长按模式
	// 			}
	// 			else
	// 			{
	// 				key->keyStatus = KEY_IDLE;
	// 			}
	// 		}
	// 		break;
	// 	}
	// 	case KEY_PLAYING:
	// 	{
	// 		if (KEY1)
	// 		{
	// 			key->keyStatus = KEY_IDLE;
	// 		}
	// 		break;
	// 	}
	// 	default:
	// 	{
	// 		break;
	// 	}
	// }
}



// void message_Update(capStatus)
// {
// 	switch ()
// }

