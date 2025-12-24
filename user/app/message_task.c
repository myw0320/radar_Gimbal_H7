//
// Created by myw04 on 2025/11/10.
//

#include "message_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
ws2812_rgb_t ws2812_rgb;

uint8_t LED_Flow_mode = 1;
void message_init(void)
{
    ws2812_rgb.r=1;
    ws2812_rgb.g=1;
    ws2812_rgb.b=1;
}

void Message_Task(void const * argument)
{
    //osDelay(100);
   // message_init();
    for ( ; ; )
    {
        WS2812_Ctrl(ws2812_rgb.r, ws2812_rgb.g, ws2812_rgb.b);
        if (ws2812_rgb.r == 255)
        {
            LED_Flow_mode = 1;
        }
        else if (ws2812_rgb.g == 255)
        {
            LED_Flow_mode = 2;
        }
        else if (ws2812_rgb.b == 255)
        {
            LED_Flow_mode = 3;
        }
        switch (LED_Flow_mode)
        {
            case 1:
            {
                ws2812_rgb.r--;
                ws2812_rgb.g++;
                break;
            }
            case 2:
            {
                ws2812_rgb.g--;
                ws2812_rgb.b++;
                break;
            }
            case 3:
            {
                ws2812_rgb.b--;
                ws2812_rgb.r++;
                break;
            }
        }
        osDelay(10);
    }
}

