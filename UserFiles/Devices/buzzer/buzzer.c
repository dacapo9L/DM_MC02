#include "buzzer.h"

void Buzzer_Init(void) {
  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Buzzer_On(float frequency, float loudness) {
  if (frequency < 0.0f || loudness < 0.0f) {
    frequency = 0;
    loudness = 0;
  }
  if (frequency > 12000.0f || loudness > 1.0f) {
    frequency = 0;
    loudness = 0;
  }
  float arr = 1000000.0f / frequency;
  __HAL_TIM_SetAutoreload(&htim12, arr);

  float ccr = arr * loudness * 0.5f;
  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, ccr);
}

void Buzzer_Off(void) { __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0); }