#include "ws2812.h"
#include <stdbool.h>
#include <string.h>

const Struct_WS2812_Color WS2812_COLOR_BLACK = {0U, 0U, 0U};
const Struct_WS2812_Color WS2812_COLOR_WHITE = {255U, 255U, 255U};
const Struct_WS2812_Color WS2812_COLOR_RED = {255U, 0U, 0U};
const Struct_WS2812_Color WS2812_COLOR_GREEN = {0U, 255U, 0U};
const Struct_WS2812_Color WS2812_COLOR_BLUE = {0U, 0U, 255U};
const Struct_WS2812_Color WS2812_COLOR_YELLOW = {255U, 255U, 0U};
const Struct_WS2812_Color WS2812_COLOR_CYAN = {0U, 255U, 255U};
const Struct_WS2812_Color WS2812_COLOR_MAGENTA = {255U, 0U, 255U};

static Struct_SPI_Manage_Object *ws2812_spi_manage_object = NULL;
static Struct_WS2812_Color ws2812_color = {0U, 0U, 0U};

static const uint8_t WS2812_LEVEL_0 = 0x60U;
static const uint8_t WS2812_LEVEL_1 = 0x78U;
static const uint16_t WS2812_FRAME_BYTES = 25U;

static float WS2812_Clamp_Brightness(float brightness) {
  if (brightness < 0.0f) {
    return 0.0f;
  }
  if (brightness > 1.0f) {
    return 1.0f;
  }
  return brightness;
}

void WS2812_Init(void) {
  SPI_Init(&hspi6, NULL);
  ws2812_spi_manage_object = &SPI6_Manage_Object;
  ws2812_color = WS2812_COLOR_BLACK;
}

void WS2812_Set_Red(uint8_t red) {
  ws2812_color.Red = red;
  ws2812_color.Green = 0U;
  ws2812_color.Blue = 0U;
}

void WS2812_Set_Green(uint8_t green) {
  ws2812_color.Red = 0U;
  ws2812_color.Green = green;
  ws2812_color.Blue = 0U;
}

void WS2812_Set_Blue(uint8_t blue) {
  ws2812_color.Red = 0U;
  ws2812_color.Green = 0U;
  ws2812_color.Blue = blue;
}

void WS2812_Set_RGB(uint8_t red, uint8_t green, uint8_t blue) {
  ws2812_color.Red = red;
  ws2812_color.Green = green;
  ws2812_color.Blue = blue;
}

void WS2812_Set_Color(Struct_WS2812_Color color, float brightness) {
  float scale = WS2812_Clamp_Brightness(brightness);
  ws2812_color.Red = (uint8_t)((float)color.Red * scale);
  ws2812_color.Green = (uint8_t)((float)color.Green * scale);
  ws2812_color.Blue = (uint8_t)((float)color.Blue * scale);
}

bool WS2812_Close(void) {
  ws2812_color.Red = 0U;
  ws2812_color.Green = 0U;
  ws2812_color.Blue = 0U;
  return true;
}

void WS2812_TIM_10ms_Write_PeriodElapsedCallback(void) {
  if (ws2812_spi_manage_object == NULL || ws2812_spi_manage_object->SPI_Handler == NULL) {
    return;
  }

  uint8_t *tmp_buffer = ws2812_spi_manage_object->Tx_Buffer;
  memset(tmp_buffer, 0, SPI_BUFFER_SIZE);

  for (uint8_t i = 0U; i < 8U; i++) {
    tmp_buffer[7U - i] =
        (ws2812_color.Green & (uint8_t)(1U << i)) ? WS2812_LEVEL_1 : WS2812_LEVEL_0;
    tmp_buffer[15U - i] =
        (ws2812_color.Red & (uint8_t)(1U << i)) ? WS2812_LEVEL_1 : WS2812_LEVEL_0;
    tmp_buffer[23U - i] =
        (ws2812_color.Blue & (uint8_t)(1U << i)) ? WS2812_LEVEL_1 : WS2812_LEVEL_0;
  }

  SPI_Transmit_Data(ws2812_spi_manage_object->SPI_Handler, NULL, 0U, GPIO_PIN_SET,
                    WS2812_FRAME_BYTES);
}
