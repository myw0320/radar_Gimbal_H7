#include "message_task.h"
#include "detect_task.h"
#include "cmsis_os.h"

ws2812_rgb_t ws2812_rgb;
buzzer_struct buzzerMessage;
void message_init(void)
{
    Buzzer_Init();
}

void Message_Task(void const * argument)
{
    message_init();
    static uint16_t current_time = 0;
    while (1)
    {
        //Buzzer_FreqUpdate(&buzzerMessage,SINGLE,startSound,16);

        current_time++;
        if (current_time >= 10)
        {
            ws2812_bink(ws2812_rgb.r, ws2812_rgb.g, ws2812_rgb.b);
            ws2812_rgb.r++;
            ws2812_rgb.g += 5;
            ws2812_rgb.b += 10;
            ws2812_rgb.r++;
            ws2812_rgb.g++;
            ws2812_rgb.b++;
            current_time = 0;
        }

        if (toe_is_error(DBUS_TOE))
        {

            //Buzzer_FreqUpdate(&buzzerMessage,SINGLE,startSound,16);
        }
        osDelay(10);
    }

}

