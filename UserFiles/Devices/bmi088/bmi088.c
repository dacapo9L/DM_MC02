#include "bmi088.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define BMI088_TRANSFER_TIMEOUT_TICK_10US 2U
#define BMI088_TEMP_PERIOD_MS 128U

static Struct_BMI088 g_bmi088 = {0};

static const float BMI088_ACCEL_AFFINE_DATA[9] = {
    0.9813826498493404f,    0.17232440504057203f, 0.027984325323801115f,
    -0.1690535919907899f,   0.9747302115792275f,  -0.10336863799715153f,
    -0.046825636945091266f, 0.09953521655990044f, 0.986897809387138f,
};
static const float BMI088_ACCEL_BIAS_DATA[3] = {
    0.0038458286072392397f, 0.00647039594993548f, 0.014968990490337293f};
static const float BMI088_GYRO_ZERO_OFFSET[3] = {
    -0.005280993487f, -0.000237223741f, -0.000647540528f};
static const float BMI088_GRAVITY_ACCELERATION = 9.7947f;

static inline bool BMI088_Is_Timeout_Tick(uint32_t now_tick,
                                          uint32_t start_tick,
                                          uint32_t timeout_tick) {
  return ((uint32_t)(now_tick - start_tick) >= timeout_tick);
}

static inline bool BMI088_SPI_Request_OK(uint8_t status) {
  return (status == (uint8_t)HAL_OK);
}

static void BMI088_Copy_Raw_Int16(const int16_t in_xyz[3], int16_t out_xyz[3]) {
  if (out_xyz == NULL) {
    return;
  }
  out_xyz[0] = in_xyz[0];
  out_xyz[1] = in_xyz[1];
  out_xyz[2] = in_xyz[2];
}

static void BMI088_Copy_Raw_Float(const float in_xyz[3], float out_xyz[3]) {
  if (out_xyz == NULL) {
    return;
  }
  out_xyz[0] = in_xyz[0];
  out_xyz[1] = in_xyz[1];
  out_xyz[2] = in_xyz[2];
}

static void BMI088_Apply_Accel_Calibration(const float in_xyz[3],
                                           float out_xyz[3]) {
  float in_norm[3];
  float calibrated[3];

  in_norm[0] = in_xyz[0] / BMI088_GRAVITY_ACCELERATION;
  in_norm[1] = in_xyz[1] / BMI088_GRAVITY_ACCELERATION;
  in_norm[2] = in_xyz[2] / BMI088_GRAVITY_ACCELERATION;

  calibrated[0] = BMI088_ACCEL_AFFINE_DATA[0] * in_norm[0] +
                  BMI088_ACCEL_AFFINE_DATA[1] * in_norm[1] +
                  BMI088_ACCEL_AFFINE_DATA[2] * in_norm[2] +
                  BMI088_ACCEL_BIAS_DATA[0];
  calibrated[1] = BMI088_ACCEL_AFFINE_DATA[3] * in_norm[0] +
                  BMI088_ACCEL_AFFINE_DATA[4] * in_norm[1] +
                  BMI088_ACCEL_AFFINE_DATA[5] * in_norm[2] +
                  BMI088_ACCEL_BIAS_DATA[1];
  calibrated[2] = BMI088_ACCEL_AFFINE_DATA[6] * in_norm[0] +
                  BMI088_ACCEL_AFFINE_DATA[7] * in_norm[1] +
                  BMI088_ACCEL_AFFINE_DATA[8] * in_norm[2] +
                  BMI088_ACCEL_BIAS_DATA[2];

  out_xyz[0] = calibrated[0] * BMI088_GRAVITY_ACCELERATION;
  out_xyz[1] = calibrated[1] * BMI088_GRAVITY_ACCELERATION;
  out_xyz[2] = calibrated[2] * BMI088_GRAVITY_ACCELERATION;
}

static void BMI088_Apply_Gyro_Calibration(const float in_xyz[3],
                                          float out_xyz[3]) {
  out_xyz[0] = in_xyz[0] + BMI088_GYRO_ZERO_OFFSET[0];
  out_xyz[1] = in_xyz[1] + BMI088_GYRO_ZERO_OFFSET[1];
  out_xyz[2] = in_xyz[2] + BMI088_GYRO_ZERO_OFFSET[2];
}

void BMI088_Init(float target_temperature_c, bool heater_enable) {
  bool accel_ok;
  bool gyro_ok;

  memset(&g_bmi088, 0, sizeof(g_bmi088));

  SPI_Init(&hspi2, BMI088_SPI_RxCpltCallback);

  g_bmi088.SPI_Manage_Object = &SPI2_Manage_Object;

  BMI088_Accel_Init(&g_bmi088.Accel, g_bmi088.SPI_Manage_Object);
  BMI088_Gyro_Init(&g_bmi088.Gyro, g_bmi088.SPI_Manage_Object);

  accel_ok = BMI088_Accel_Configure(&g_bmi088.Accel);
  gyro_ok = BMI088_Gyro_Configure(&g_bmi088.Gyro);

  g_bmi088.init_finished_flag = (accel_ok && gyro_ok);
  g_bmi088.calibration_enable = false;

  BMI088_Heater_Init();
  BMI088_Heater_Set_Enable(heater_enable);
  BMI088_Heater_Set_Target_Temperature_C(target_temperature_c);
}

void BMI088_EXTI_Callback(uint16_t gpio_pin) {
  if (!g_bmi088.init_finished_flag) {
    return;
  }

  if (gpio_pin == BMI088_ACCEL_INTERRUPT_Pin) {
    g_bmi088.accel_ready_flag = true;
  } else if (gpio_pin == BMI088_GYRO_INTERRUPT_Pin) {
    g_bmi088.gyro_ready_flag = true;
  }
}

void BMI088_TIM_1ms_PeriodElapsedCallback(void) {
  if (!g_bmi088.init_finished_flag) {
    return;
  }

  g_bmi088.temp_1ms_counter++;
  if (g_bmi088.temp_1ms_counter >= BMI088_TEMP_PERIOD_MS) {
    g_bmi088.temp_1ms_counter = 0U;
    g_bmi088.temp_ready_flag = true;
  }
}

void BMI088_TIM_10us_PeriodElapsedCallback(void) {
  uint8_t spi_status;

  if (!g_bmi088.init_finished_flag) {
    return;
  }

  g_bmi088.sched_10us_tick++;

  if (g_bmi088.accel_ready_flag && !g_bmi088.accel_transfering_flag &&
      !g_bmi088.gyro_transfering_flag && !g_bmi088.temp_transfering_flag) {
    spi_status = BMI088_Accel_Request_Accel(&g_bmi088.Accel);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.accel_transfering_flag = true;
      g_bmi088.accel_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.accel_ready_flag = false;
    } else {
      g_bmi088.accel_transfering_flag = false;
      g_bmi088.accel_ready_flag = true;
    }
    return;
  }

  if (g_bmi088.gyro_ready_flag && !g_bmi088.accel_transfering_flag &&
      !g_bmi088.gyro_transfering_flag && !g_bmi088.temp_transfering_flag) {
    spi_status = BMI088_Gyro_Request_Gyro(&g_bmi088.Gyro);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.gyro_transfering_flag = true;
      g_bmi088.gyro_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.gyro_ready_flag = false;
    } else {
      g_bmi088.gyro_transfering_flag = false;
      g_bmi088.gyro_ready_flag = true;
    }
    return;
  }

  if (g_bmi088.temp_ready_flag && !g_bmi088.accel_transfering_flag &&
      !g_bmi088.gyro_transfering_flag && !g_bmi088.temp_transfering_flag) {
    spi_status = BMI088_Accel_Request_Temperature(&g_bmi088.Accel);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.temp_transfering_flag = true;
      g_bmi088.temp_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.temp_ready_flag = false;
    } else {
      g_bmi088.temp_transfering_flag = false;
      g_bmi088.temp_ready_flag = true;
    }
    return;
  }

  if (g_bmi088.accel_transfering_flag &&
      BMI088_Is_Timeout_Tick(g_bmi088.sched_10us_tick,
                             g_bmi088.accel_transfering_start_tick,
                             BMI088_TRANSFER_TIMEOUT_TICK_10US)) {
    spi_status = BMI088_Accel_Request_Accel(&g_bmi088.Accel);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.accel_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.accel_ready_flag = false;
    } else {
      g_bmi088.accel_transfering_flag = false;
      g_bmi088.accel_ready_flag = true;
    }
    return;
  }

  if (g_bmi088.gyro_transfering_flag &&
      BMI088_Is_Timeout_Tick(g_bmi088.sched_10us_tick,
                             g_bmi088.gyro_transfering_start_tick,
                             BMI088_TRANSFER_TIMEOUT_TICK_10US)) {
    spi_status = BMI088_Gyro_Request_Gyro(&g_bmi088.Gyro);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.gyro_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.gyro_ready_flag = false;
    } else {
      g_bmi088.gyro_transfering_flag = false;
      g_bmi088.gyro_ready_flag = true;
    }
    return;
  }

  if (g_bmi088.temp_transfering_flag &&
      BMI088_Is_Timeout_Tick(g_bmi088.sched_10us_tick,
                             g_bmi088.temp_transfering_start_tick,
                             BMI088_TRANSFER_TIMEOUT_TICK_10US)) {
    spi_status = BMI088_Accel_Request_Temperature(&g_bmi088.Accel);
    if (BMI088_SPI_Request_OK(spi_status)) {
      g_bmi088.temp_transfering_start_tick = g_bmi088.sched_10us_tick;
      g_bmi088.temp_ready_flag = false;
    } else {
      g_bmi088.temp_transfering_flag = false;
      g_bmi088.temp_ready_flag = true;
    }
    return;
  }
}

void BMI088_SPI_RxCpltCallback(uint8_t *tx, uint8_t *rx, uint16_t tx_len,
                               uint16_t rx_len) {
  uint8_t reg;

  if (!g_bmi088.init_finished_flag || tx == NULL || rx == NULL ||
      tx_len == 0U || g_bmi088.SPI_Manage_Object == NULL) {
    return;
  }

  if (g_bmi088.SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL_CS_GPIO_Port &&
      g_bmi088.SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL_CS_Pin) {
    BMI088_Accel_SPI_RxCpltCallback(&g_bmi088.Accel, tx, rx, tx_len, rx_len);

    reg = (uint8_t)(tx[0] & (uint8_t)(~BMI088_ACCEL_READ_MASK));
    if (reg == BMI088_ACCEL_REG_ACC_X_LSB && rx_len == 6U) {
      g_bmi088.accel_transfering_flag = false;
      g_bmi088.accel_ready_flag = true;
    } else if (reg == BMI088_ACCEL_REG_TEMP_MSB && rx_len == 2U) {
      g_bmi088.temp_transfering_flag = false;
    }
  } else if (g_bmi088.SPI_Manage_Object->Activate_GPIOx ==
                 BMI088_GYRO_CS_GPIO_Port &&
             g_bmi088.SPI_Manage_Object->Activate_GPIO_Pin ==
                 BMI088_GYRO_CS_Pin) {
    BMI088_Gyro_SPI_RxCpltCallback(&g_bmi088.Gyro, tx, rx, tx_len, rx_len);
    g_bmi088.gyro_transfering_flag = false;
    g_bmi088.gyro_ready_flag = true;
  }
}

void BMI088_Set_Calibration_Enable(bool enable) {
  g_bmi088.calibration_enable = enable;
}

bool BMI088_Get_Calibration_Enable(void) { return g_bmi088.calibration_enable; }

bool BMI088_Get_Accel_Valid(void) { return g_bmi088.Accel.valid_flag; }

bool BMI088_Get_Gyro_Valid(void) { return g_bmi088.Gyro.valid_flag; }

float BMI088_Get_Temperature_C(void) { return g_bmi088.Accel.temperature_c; }

void BMI088_Get_Accel_Raw(int16_t out_xyz[3]) {
  BMI088_Copy_Raw_Int16(g_bmi088.Accel.raw_accel, out_xyz);
}

void BMI088_Get_Gyro_Raw(int16_t out_xyz[3]) {
  BMI088_Copy_Raw_Int16(g_bmi088.Gyro.raw_gyro, out_xyz);
}

void BMI088_Get_Accel_mps2(float out_xyz[3]) {
  if (out_xyz == NULL) {
    return;
  }

  if (!g_bmi088.calibration_enable) {
    BMI088_Copy_Raw_Float(g_bmi088.Accel.accel_mps2, out_xyz);
  } else {
    BMI088_Apply_Accel_Calibration(g_bmi088.Accel.accel_mps2, out_xyz);
  }
}

void BMI088_Get_Gyro_rads(float out_xyz[3]) {
  if (out_xyz == NULL) {
    return;
  }

  if (!g_bmi088.calibration_enable) {
    BMI088_Copy_Raw_Float(g_bmi088.Gyro.gyro_rads, out_xyz);
  } else {
    BMI088_Apply_Gyro_Calibration(g_bmi088.Gyro.gyro_rads, out_xyz);
  }
}
