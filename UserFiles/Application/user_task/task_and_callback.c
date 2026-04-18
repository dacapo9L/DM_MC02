#include "task_and_callback.h"
#include <string.h>

bool init_finished = false;

void CAN1_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer) {
  if (Header == NULL || Buffer == NULL) {
    return;
  }
  if (!init_finished) {
    return;
  }

  uint16_t id = (uint16_t)Header->Identifier;
  uint8_t index = (uint8_t)(id - 0x201U);
  if (id >= 0x201U && id <= 0x20BU && dji_motor_can1map[index] != NULL) {
    DJI_Motor_CAN_RxCpltCallback(dji_motor_can1map[index]);
  }
}

void CAN2_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer) {
  uint8_t id;
  uint8_t index;

  if (Header == NULL || Buffer == NULL) {
    return;
  }
  if (!init_finished) {
    return;
  }
  if (Header->IdType != FDCAN_EXTENDED_ID) {
    return;
  }

  id = (uint8_t)((Header->Identifier >> 8U) & 0xFFU);
  switch (id) {
  case 0x01:
    index = 0U;
    break;
  case 0x03:
    index = 1U;
    break;
  case 0x04:
    index = 2U;
    break;
  case 0x05:
    index = 3U;
    break;
  default:
    return;
  }

  if (zdt_stepper_can2map[index] != NULL) {
    ZDT_Stepper_CAN_RxCpltCallback(zdt_stepper_can2map[index]);
  }
}

void USART1_Callback(UART_HandleTypeDef *huart, uint8_t *Buffer, uint16_t Size) {
  uint16_t tx_len;

  if (huart == NULL || Buffer == NULL) {
    return;
  }
  if (!init_finished) {
    return;
  }
  if (Size == 0U || USART1_Manage_Object.USART_Handler == NULL) {
    return;
  }

  tx_len = Size;
  if (tx_len > (uint16_t)sizeof(USART1_Manage_Object.Tx_Buffer)) {
    tx_len = (uint16_t)sizeof(USART1_Manage_Object.Tx_Buffer);
  }

  memcpy(USART1_Manage_Object.Tx_Buffer, Buffer, tx_len);
  HAL_UART_Transmit_DMA(USART1_Manage_Object.USART_Handler,
                        USART1_Manage_Object.Tx_Buffer, tx_len);
}

void Task_Init(void) {
  CAN_Init(&hfdcan1, false, CAN1_Callback);
  CAN_Init(&hfdcan2, true, CAN2_Callback);

  USART_Init(&huart1, USART1_Callback);

  USB_Init(NULL);
  INS_Init();

  Buzzer_Init();
  WS2812_Init();
  BMI088_Init(65.0f, true);

  init_finished = true;

  if (init_finished) {
    // Buzzer_On(880.000f, 0.8f);
    WS2812_Set_Color(WS2812_COLOR_GREEN, 0.15f);
    WS2812_Write_Callback();
  } else {
    // Buzzer_On(392.000f, 0.8f);
    WS2812_Set_Color(WS2812_COLOR_RED, 0.15f);
    WS2812_Write_Callback();
  }

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim8);
}

void Task3600s_Callback() {}

void Task1s_Callback() {
  static uint8_t close_tick = 0;
  close_tick++;
  if (close_tick >= 1U) {
    Buzzer_Off();
    WS2812_Set_RGB(0, 0, 0);
    WS2812_Write_Callback();
    close_tick = 0U;
  }
}

void Task1ms_Callback() {
  static uint16_t alive_tick = 0;
  static uint16_t heater_tick = 0;

  TIM_1ms_CAN_PeriodElapsedCallback();

  alive_tick++;
  if (alive_tick >= 100U) {
    alive_tick = 0U;
    DJI_Motor_TIM_100ms_Alive_PeriodElapsedCallback(NULL);
    // ZDT_Stepper_TIM_100ms_Alive_PeriodElapsedCallback(NULL);
  }

  BMI088_TIM_1ms_PeriodElapsedCallback();
  heater_tick++;
  if (heater_tick >= 128U) {
    heater_tick = 0U;
    BMI088_Heater_TIM_128ms_PeriodElapsedCallback();
  }
}

void Task125us_Callback() {}

void Task10us_Callback() { BMI088_TIM_10us_PeriodElapsedCallback(); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (!init_finished) {
    return;
  }
  BMI088_EXTI_Callback(GPIO_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (!init_finished) {
    return;
  }

  if (htim->Instance == TIM4) {
    Task10us_Callback();
  } else if (htim->Instance == TIM5) {
    Task3600s_Callback();
  } else if (htim->Instance == TIM6) {
    Task1s_Callback();
  } else if (htim->Instance == TIM7) {
    Task1ms_Callback();
  } else if (htim->Instance == TIM8) {
    Task125us_Callback();
  }
}
