//
// Created by myw04 on 2025/11/10.
//

#include "message_task.h"

#include "cmsis_os.h"

static uint8_t r = 1;
static uint8_t g = 1;
static uint8_t b = 1;

void message_init(void)
{

}

void Message_Task(void const * argument)
{
    ws2812_bink(r, g, b);
    r++;
    g += 5;
    b += 10;
    HAL_Delay(1);
    r++;g++;b++;
    osDelay(300);

}

