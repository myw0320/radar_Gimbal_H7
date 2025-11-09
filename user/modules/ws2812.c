//
// Created by myw04 on 2025/11/7.
//

#include "ws2812.h"
#include "tim.h"
rgb_colors_struct RED     = {50,0,0};
rgb_colors_struct GREEN   = {0,50,0};
rgb_colors_struct BLUE    = {0,0,50};
rgb_colors_struct MAGENTA = {50,0,50};
rgb_colors_struct BLACK   = {0,0,0};
rgb_colors_struct WHILTE = {100,100,100};
/*二维数组存放最终PWM输出数组，每一行24个数据代表一个LED，最后一行24个0代表RESET码*/
uint32_t Pixel_Buf[RGB_NUM+1][55];


void RGB_SetColor(uint8_t LedId,rgb_colors_struct Color)
{
    uint8_t i;
    for(i=0;i<8;i++) Pixel_Buf[LedId][i]   = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
    for(i=8;i<16;i++) Pixel_Buf[LedId][i]  = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
    for(i=16;i<24;i++) Pixel_Buf[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}

void Reset_Load(void)
{
    uint8_t i;
    for(i=0;i<24;i++)
    {
        Pixel_Buf[RGB_NUM][i] = 0;
    }
}


void RGB_SendArray(void)
{
    //HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, (uint32_t *)Pixel_Buf,(RGB_NUM+1)*24);
}

//
void RGB_Colors(uint8_t rgb_len,rgb_colors_struct colors)
{
    for(uint8_t i=0;i<rgb_len;i++)//给对应个数LED写入红色
    {
        RGB_SetColor(i,colors);
        Reset_Load();
        RGB_SendArray();
    }

}

