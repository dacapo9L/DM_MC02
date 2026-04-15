#ifndef BMI088_HEATER_H
#define BMI088_HEATER_H

#include <stdbool.h>
#include <stdint.h>

void BMI088_Heater_Init(void);

void BMI088_Heater_Set_Enable(bool enable);
bool BMI088_Heater_Get_Enable(void);

void BMI088_Heater_Set_Target_Temperature_C(float target_c);
float BMI088_Heater_Get_Target_Temperature_C(void);

float BMI088_Heater_Get_Now_Temperature_C(void);

void BMI088_Heater_Set_Supply_Voltage(float voltage_v);
float BMI088_Heater_Get_Supply_Voltage(void);

uint32_t BMI088_Heater_Get_Output_Compare(void);

void BMI088_Heater_TIM_128ms_PeriodElapsedCallback(void);

#endif
