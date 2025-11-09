#ifndef RADAR_GIMBAL_WS2812_H
#define RADAR_GIMBAL_WS2812_H
#include "main.h"

/****RGB****/
#define CODE_1       (58)       //1码定时器计数次数
#define CODE_0       (25)       //0码定时器计数次数
#define RGB_NUM    3	// RGB灯的数量，即为缓冲区长度
typedef struct			//颜色结构体
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
}rgb_colors_struct;
extern rgb_colors_struct RED;
extern rgb_colors_struct GREEN;
extern rgb_colors_struct BLUE;
extern rgb_colors_struct MAGENTA;
// 各种颜色测试
void RGB_Colors(uint8_t rgb_len,rgb_colors_struct colors);

#endif //RADAR_GIMBAL_WS2812_H