#include "task_and_callback.h"

// Global init-finished flag.
static bool init_finished = false;

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

void Task_Init(void) {
  CAN_Init(&hfdcan1, CAN1_Callback);
  USB_Init(NULL);

  Buzzer_Init();
  WS2812_Init();

  init_finished = true;

  if (init_finished) {
    // Buzzer_On(880.000f, 0.8f);
    WS2812_Set_Color(WS2812_COLOR_GREEN, 0.15f);
    WS2812_TIM_10ms_Write_PeriodElapsedCallback();
  } else {
    // Buzzer_On(392.000f, 0.8f);
    WS2812_Set_Color(WS2812_COLOR_RED, 0.15f);
    WS2812_TIM_10ms_Write_PeriodElapsedCallback();
  }

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim8);
}

void Task3600s_Callback() {}

void Task1s_Callback() {
  static uint8_t buzzer_close_tick = 0;
  buzzer_close_tick++;
  if (buzzer_close_tick >= 1U) {
    Buzzer_Off();
    WS2812_Close();
    WS2812_TIM_10ms_Write_PeriodElapsedCallback();
    buzzer_close_tick = 0U;
  }
}

void Task1ms_Callback() {
  static uint16_t alive_tick = 0;

  TIM_1ms_CAN_PeriodElapsedCallback();

  alive_tick++;
  if (alive_tick >= 100U) {
    alive_tick = 0U;
    DJI_Motor_TIM_100ms_Alive_PeriodElapsedCallback(NULL);
  }
}

void Task125us_Callback() {}

void Task10us_Callback() {}

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
