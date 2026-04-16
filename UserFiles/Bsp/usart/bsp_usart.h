#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "usart.h"
#include <stdbool.h>

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

extern bool init_finished;

typedef void (*USART_Callback)(UART_HandleTypeDef *USART_Handler,
                               uint8_t *Buffer, uint16_t Length);

typedef struct Struct_USART_Manage_Object {
  UART_HandleTypeDef *USART_Handler;
  DMA_HandleTypeDef *DMA_Handler_Rx;
  DMA_HandleTypeDef *DMA_Handler_Tx;
  USART_Callback Callback_Function;
  uint8_t Rx_Buffer[512];
  uint8_t Tx_Buffer[64];
  bool Rx_Complete_Flag;
  uint8_t Rx_Data_Length;
} Struct_USART_Manage_Object;

extern Struct_USART_Manage_Object USART1_Manage_Object;
extern Struct_USART_Manage_Object USART2_Manage_Object;
extern Struct_USART_Manage_Object USART3_Manage_Object;
extern Struct_USART_Manage_Object UART7_Manage_Object;
extern Struct_USART_Manage_Object USART10_Manage_Object;

void USART_Init(UART_HandleTypeDef *huart, USART_Callback Callback_Function);
void USART_Transmit_Data(Struct_USART_Manage_Object *USART_Manage_Object);

#endif
