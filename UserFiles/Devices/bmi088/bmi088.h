#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>

void BMI088_Init(float target_temperature_c, bool heater_enable);

void BMI088_EXTI_Callback(uint16_t gpio_pin);
void BMI088_TIM_10us_PeriodElapsedCallback(void);
void BMI088_TIM_1ms_PeriodElapsedCallback(void);
void BMI088_SPI_RxCpltCallback(uint8_t *tx, uint8_t *rx, uint16_t tx_len,
                               uint16_t rx_len);

void BMI088_Set_Calibration_Enable(bool enable);
bool BMI088_Get_Calibration_Enable(void);

bool BMI088_Get_Accel_Valid(void);
bool BMI088_Get_Gyro_Valid(void);
float BMI088_Get_Temperature_C(void);

void BMI088_Get_Accel_Raw(int16_t out_xyz[3]);
void BMI088_Get_Gyro_Raw(int16_t out_xyz[3]);
void BMI088_Get_Accel_mps2(float out_xyz[3]);
void BMI088_Get_Gyro_rads(float out_xyz[3]);

#endif
