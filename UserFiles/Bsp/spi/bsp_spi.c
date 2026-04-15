#include "bsp_spi.h"
#include "bsp_dwt.h"
#include <string.h>

/* DMA1/DMA2 cannot access DTCMRAM on STM32H7, place SPI buffers in AXI SRAM */
#define SPI_DMA_BUFFER_ATTR __attribute__((section(".dma_buffer")))

SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI1_Manage_Object;
SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI2_Manage_Object;
SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI3_Manage_Object;
SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI4_Manage_Object;
SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI5_Manage_Object;
SPI_DMA_BUFFER_ATTR Struct_SPI_Manage_Object SPI6_Manage_Object;

static Struct_SPI_Manage_Object *
SPI_Get_Manage_Object(SPI_HandleTypeDef *hspi) {
  if (hspi == NULL) {
    return NULL;
  }

  if (hspi->Instance == SPI1) {
    return &SPI1_Manage_Object;
  }
  if (hspi->Instance == SPI2) {
    return &SPI2_Manage_Object;
  }
  if (hspi->Instance == SPI3) {
    return &SPI3_Manage_Object;
  }
  if (hspi->Instance == SPI4) {
    return &SPI4_Manage_Object;
  }
  if (hspi->Instance == SPI5) {
    return &SPI5_Manage_Object;
  }
  if (hspi->Instance == SPI6) {
    return &SPI6_Manage_Object;
  }

  return NULL;
}

static void SPI_Set_CS(const Struct_SPI_Manage_Object *obj,
                       GPIO_PinState level) {
  if (obj != NULL && obj->Activate_GPIOx != NULL) {
    HAL_GPIO_WritePin(obj->Activate_GPIOx, obj->Activate_GPIO_Pin, level);
  }
}

static void SPI_Deactivate_CS(const Struct_SPI_Manage_Object *obj) {
  if (obj == NULL || obj->Activate_GPIOx == NULL) {
    return;
  }

  if (obj->Activate_Level == GPIO_PIN_SET) {
    HAL_GPIO_WritePin(obj->Activate_GPIOx, obj->Activate_GPIO_Pin,
                      GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(obj->Activate_GPIOx, obj->Activate_GPIO_Pin,
                      GPIO_PIN_SET);
  }
}

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function) {
  Struct_SPI_Manage_Object *obj = SPI_Get_Manage_Object(hspi);
  if (obj == NULL) {
    return;
  }

  memset(obj, 0, sizeof(*obj));
  obj->SPI_Handler = hspi;
  obj->Callback_Function = Callback_Function;
}

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                          uint16_t GPIO_Pin, GPIO_PinState Activate_Level,
                          uint16_t Tx_Length) {
  Struct_SPI_Manage_Object *obj = SPI_Get_Manage_Object(hspi);
  uint8_t status;
  if (obj == NULL) {
    return HAL_ERROR;
  }

  obj->Activate_GPIOx = GPIOx;
  obj->Activate_GPIO_Pin = GPIO_Pin;
  obj->Activate_Level = Activate_Level;
  obj->Tx_Buffer_Length = Tx_Length;
  obj->Rx_Buffer_Length = 0;

  SPI_Set_CS(obj, Activate_Level);

  if (hspi->Instance == SPI6) {
    status = (uint8_t)HAL_SPI_Transmit(hspi, obj->Tx_Buffer, Tx_Length, 1);
  } else {
    status = (uint8_t)HAL_SPI_Transmit_DMA(hspi, obj->Tx_Buffer, Tx_Length);
  }

  if (status != HAL_OK) {
    SPI_Deactivate_CS(obj);
    obj->Activate_GPIOx = NULL;
  }

  return status;
}

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                                  uint16_t GPIO_Pin,
                                  GPIO_PinState Activate_Level,
                                  uint16_t Tx_Length, uint16_t Rx_Length) {
  Struct_SPI_Manage_Object *obj = SPI_Get_Manage_Object(hspi);
  uint8_t status;
  if (obj == NULL) {
    return HAL_ERROR;
  }

  obj->Activate_GPIOx = GPIOx;
  obj->Activate_GPIO_Pin = GPIO_Pin;
  obj->Activate_Level = Activate_Level;
  obj->Tx_Buffer_Length = Tx_Length;
  obj->Rx_Buffer_Length = Rx_Length;

  SPI_Set_CS(obj, Activate_Level);

  if (hspi->Instance == SPI6) {
    status = (uint8_t)HAL_SPI_TransmitReceive(
        hspi, obj->Tx_Buffer, obj->Rx_Buffer, Tx_Length + Rx_Length, 1);
  } else {
    status = (uint8_t)HAL_SPI_TransmitReceive_DMA(
        hspi, obj->Tx_Buffer, obj->Rx_Buffer, Tx_Length + Rx_Length);
  }

  if (status != HAL_OK) {
    SPI_Deactivate_CS(obj);
    obj->Activate_GPIOx = NULL;
  }

  return status;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  Struct_SPI_Manage_Object *obj = SPI_Get_Manage_Object(hspi);
  if (obj == NULL) {
    return;
  }

  SPI_Deactivate_CS(obj);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  Struct_SPI_Manage_Object *obj = SPI_Get_Manage_Object(hspi);
  if (obj == NULL) {
    return;
  }

  SPI_Deactivate_CS(obj);
  obj->Rx_Timestamp = (uint64_t)DWT_Get_Timestamp();

  if (obj->Callback_Function != NULL) {
    obj->Callback_Function(obj->Tx_Buffer, obj->Rx_Buffer,
                           obj->Tx_Buffer_Length, obj->Rx_Buffer_Length);
  }

  obj->Activate_GPIOx = NULL;
}
