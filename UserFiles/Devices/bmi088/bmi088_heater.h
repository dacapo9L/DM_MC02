#ifndef BMI088_HEATER_H
#define BMI088_HEATER_H

#include "pid.h"
#include "tim.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool initialized;
  bool enable;
  bool preheat_finished;

  float now_temperature_c;
  float target_temperature_c;
  float supply_voltage_v;

  uint32_t output_compare;
  PID_t pid;
} BMI088_Heater_State_t;

void BMI088_Heater_Init(void);
void BMI088_Heater_Set_Enable(bool enable);
void BMI088_Heater_Set_Target_Temperature_C(float temperature_c);
void BMI088_Heater_TIM_128ms_PeriodElapsedCallback(void);

#endif
