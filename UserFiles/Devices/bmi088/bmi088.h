#ifndef BMI088_H
#define BMI088_H

#include "bmi088_accel.h"
#include "bmi088_gyro.h"
#include "bmi088_heater.h"
#include "bsp_spi.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct Struct_BMI088 {
  Struct_SPI_Manage_Object *SPI_Manage_Object;
  Struct_BMI088_Accel Accel;
  Struct_BMI088_Gyro Gyro;

  bool init_finished_flag;
  bool calibration_enable;

  bool accel_ready_flag;
  bool gyro_ready_flag;
  bool temp_ready_flag;

  bool accel_transfering_flag;
  bool gyro_transfering_flag;
  bool temp_transfering_flag;

  bool accel_update_flag;
  bool gyro_update_flag;

  uint32_t sched_10us_tick;
  uint32_t accel_transfering_start_tick;
  uint32_t gyro_transfering_start_tick;
  uint32_t temp_transfering_start_tick;
  uint8_t temp_1ms_counter;
} Struct_BMI088;

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
