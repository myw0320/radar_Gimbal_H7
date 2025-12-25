#ifndef __WS2812_H__
#define __WS2812_H__
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
}ws2812_rgb_t;
void ws2812_bink(uint8_t r, uint8_t g, uint8_t b);
#endif
