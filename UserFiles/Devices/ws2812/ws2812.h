#ifndef WS2812_H
#define WS2812_H

#include "bsp_spi.h"
#include "main.h"
#include <stdbool.h>

typedef struct {
  uint8_t Red;
  uint8_t Green;
  uint8_t Blue;
} Struct_WS2812_Color;

extern const Struct_WS2812_Color WS2812_COLOR_BLACK;
extern const Struct_WS2812_Color WS2812_COLOR_WHITE;
extern const Struct_WS2812_Color WS2812_COLOR_RED;
extern const Struct_WS2812_Color WS2812_COLOR_GREEN;
extern const Struct_WS2812_Color WS2812_COLOR_BLUE;
extern const Struct_WS2812_Color WS2812_COLOR_YELLOW;
extern const Struct_WS2812_Color WS2812_COLOR_CYAN;
extern const Struct_WS2812_Color WS2812_COLOR_MAGENTA;

void WS2812_Init(void);

void WS2812_Set_Red(uint8_t red);
void WS2812_Set_Green(uint8_t green);
void WS2812_Set_Blue(uint8_t blue);
void WS2812_Set_RGB(uint8_t red, uint8_t green, uint8_t blue);
void WS2812_Set_Color(Struct_WS2812_Color color, float brightness);
bool WS2812_Close(void);

void WS2812_TIM_10ms_Write_PeriodElapsedCallback(void);

#endif // WS2812_H
