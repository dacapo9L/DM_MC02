#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"
#include "spi.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#define SPI_BUFFER_SIZE 512U

typedef void (*SPI_Callback)(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer,
                             uint16_t Tx_Length, uint16_t Rx_Length);

typedef struct Struct_SPI_Manage_Object {
  SPI_HandleTypeDef *SPI_Handler;
  SPI_Callback Callback_Function;

  GPIO_TypeDef *Activate_GPIOx;
  uint16_t Activate_GPIO_Pin;
  GPIO_PinState Activate_Level;

  uint8_t Tx_Buffer[SPI_BUFFER_SIZE];
  uint8_t Rx_Buffer[SPI_BUFFER_SIZE];
  uint16_t Tx_Buffer_Length;
  uint16_t Rx_Buffer_Length;

  uint64_t Rx_Timestamp;
} Struct_SPI_Manage_Object;

extern Struct_SPI_Manage_Object SPI1_Manage_Object;
extern Struct_SPI_Manage_Object SPI2_Manage_Object;
extern Struct_SPI_Manage_Object SPI3_Manage_Object;
extern Struct_SPI_Manage_Object SPI4_Manage_Object;
extern Struct_SPI_Manage_Object SPI5_Manage_Object;
extern Struct_SPI_Manage_Object SPI6_Manage_Object;

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function);

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                          uint16_t GPIO_Pin, GPIO_PinState Activate_Level,
                          uint16_t Tx_Length);

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                                  uint16_t GPIO_Pin,
                                  GPIO_PinState Activate_Level,
                                  uint16_t Tx_Length, uint16_t Rx_Length);

#endif
