#include "bsp_usart.h"
#include "task_and_callback.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define UART_BUFFER_SIZE 512
#define USART_DMA_BUFFER_ATTR __attribute__((section(".dma_buffer")))

USART_DMA_BUFFER_ATTR Struct_USART_Manage_Object USART1_Manage_Object;
USART_DMA_BUFFER_ATTR Struct_USART_Manage_Object USART2_Manage_Object;
USART_DMA_BUFFER_ATTR Struct_USART_Manage_Object USART3_Manage_Object;
USART_DMA_BUFFER_ATTR Struct_USART_Manage_Object UART7_Manage_Object;
USART_DMA_BUFFER_ATTR Struct_USART_Manage_Object USART10_Manage_Object;

void USART_Init(UART_HandleTypeDef *huart, USART_Callback Callback_Function) {
  if (huart->Instance == USART1) {
    memset(&USART1_Manage_Object, 0, sizeof(USART1_Manage_Object));
    USART1_Manage_Object.USART_Handler = huart;
    USART1_Manage_Object.DMA_Handler_Rx = &hdma_usart1_rx;
    USART1_Manage_Object.DMA_Handler_Tx = &hdma_usart1_tx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART1_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
    USART1_Manage_Object.Callback_Function = Callback_Function;
  } else if (huart->Instance == USART2) {
    memset(&USART2_Manage_Object, 0, sizeof(USART2_Manage_Object));
    USART2_Manage_Object.USART_Handler = huart;
    USART2_Manage_Object.DMA_Handler_Rx = &hdma_usart2_rx;
    USART2_Manage_Object.DMA_Handler_Tx = &hdma_usart2_tx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART2_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
    USART2_Manage_Object.Callback_Function = Callback_Function;
  } else if (huart->Instance == USART3) {
    memset(&USART3_Manage_Object, 0, sizeof(USART3_Manage_Object));
    USART3_Manage_Object.USART_Handler = huart;
    USART3_Manage_Object.DMA_Handler_Rx = &hdma_usart3_rx;
    USART3_Manage_Object.DMA_Handler_Tx = &hdma_usart3_tx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART3_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
    USART3_Manage_Object.Callback_Function = Callback_Function;
  } else if (huart->Instance == UART7) {
    memset(&UART7_Manage_Object, 0, sizeof(UART7_Manage_Object));
    UART7_Manage_Object.USART_Handler = huart;
    UART7_Manage_Object.DMA_Handler_Rx = &hdma_uart7_rx;
    UART7_Manage_Object.DMA_Handler_Tx = &hdma_uart7_tx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
    UART7_Manage_Object.Callback_Function = Callback_Function;
  } else if (huart->Instance == USART10) {
    memset(&USART10_Manage_Object, 0, sizeof(USART10_Manage_Object));
    USART10_Manage_Object.USART_Handler = huart;
    USART10_Manage_Object.DMA_Handler_Rx = &hdma_usart10_rx;
    USART10_Manage_Object.DMA_Handler_Tx = &hdma_usart10_tx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART10_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
    USART10_Manage_Object.Callback_Function = Callback_Function;
  }
}

void USART_Transmit_Data(Struct_USART_Manage_Object *USART_Manage_Object) {
  HAL_UART_Transmit_DMA(USART_Manage_Object->USART_Handler,
                        USART_Manage_Object->Tx_Buffer,
                        sizeof(USART_Manage_Object->Tx_Buffer));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // 判断程序初始化完成
  if (!init_finished) {
    // 重启接收
    if (huart->Instance == USART1) {
      HAL_UARTEx_ReceiveToIdle_DMA(huart, USART1_Manage_Object.Rx_Buffer,
                                   UART_BUFFER_SIZE);
    } else if (huart->Instance == USART2) {
      HAL_UARTEx_ReceiveToIdle_DMA(huart, USART2_Manage_Object.Rx_Buffer,
                                   UART_BUFFER_SIZE);
    } else if (huart->Instance == USART3) {
      HAL_UARTEx_ReceiveToIdle_DMA(huart, USART3_Manage_Object.Rx_Buffer,
                                   UART_BUFFER_SIZE);
    } else if (huart->Instance == UART7) {
      HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer,
                                   UART_BUFFER_SIZE);
    } else if (huart->Instance == USART10) {
      HAL_UARTEx_ReceiveToIdle_DMA(huart, USART10_Manage_Object.Rx_Buffer,
                                   UART_BUFFER_SIZE);
    }
    return;
  }

  // 选择回调函数
  if (huart->Instance == USART1) {
    if (USART1_Manage_Object.Callback_Function != NULL) {
      USART1_Manage_Object.Callback_Function(
          huart, USART1_Manage_Object.Rx_Buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART1_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
  } else if (huart->Instance == USART2) {
    if (USART2_Manage_Object.Callback_Function != NULL) {
      USART2_Manage_Object.Callback_Function(
          huart, USART2_Manage_Object.Rx_Buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART2_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
  } else if (huart->Instance == USART3) {
    if (USART3_Manage_Object.Callback_Function != NULL) {
      USART3_Manage_Object.Callback_Function(
          huart, USART3_Manage_Object.Rx_Buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART3_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
  } else if (huart->Instance == UART7) {
    if (UART7_Manage_Object.Callback_Function != NULL) {
      UART7_Manage_Object.Callback_Function(
          huart, UART7_Manage_Object.Rx_Buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
  } else if (huart->Instance == USART10) {
    if (USART10_Manage_Object.Callback_Function != NULL) {
      USART10_Manage_Object.Callback_Function(
          huart, USART10_Manage_Object.Rx_Buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, USART10_Manage_Object.Rx_Buffer,
                                 UART_BUFFER_SIZE);
  }
}
