#include "bmi088_heater.h"
#include "bmi088.h"

#define BMI088_HEATER_TIM_HANDLE (&htim3)
#define BMI088_HEATER_TIM_CHANNEL TIM_CHANNEL_4

#define BMI088_HEATER_DEFAULT_TARGET_C 50.0f
#define BMI088_HEATER_PREHEAT_BASE_TEMPERATURE_C 45.0f
#define BMI088_HEATER_MAX_SAFE_TEMPERATURE_C 65.0f

#define BMI088_HEATER_PREHEAT_POWER 1000.0f
#define BMI088_HEATER_NOMINAL_VOLTAGE 25.2f

#define BMI088_HEATER_PID_KP 100.0f
#define BMI088_HEATER_PID_KI 10.0f
#define BMI088_HEATER_PID_KD 0.0f
#define BMI088_HEATER_PID_MAXOUT 300.0f
#define BMI088_HEATER_PID_I_LIMIT 500.0f
#define BMI088_HEATER_PID_DT 0.128f

static BMI088_Heater_State_t s_heater = {0};

static float BMI088_Heater_Clamp_Float(float value, float min_val,
                                       float max_val) {
  if (value < min_val) {
    return min_val;
  }
  if (value > max_val) {
    return max_val;
  }
  return value;
}

static uint32_t BMI088_Heater_Get_Max_Compare(void) {
  return (uint32_t)(__HAL_TIM_GET_AUTORELOAD(BMI088_HEATER_TIM_HANDLE) + 1U);
}

static void BMI088_Heater_Set_Compare(uint32_t compare) {
  uint32_t max_compare = BMI088_Heater_Get_Max_Compare();

  if (compare > max_compare) {
    compare = max_compare;
  }

  s_heater.output_compare = compare;
  __HAL_TIM_SET_COMPARE(BMI088_HEATER_TIM_HANDLE, BMI088_HEATER_TIM_CHANNEL,
                        compare);
}

void BMI088_Heater_Init(void) {
  pid_config_t cfg;

  cfg.kp = BMI088_HEATER_PID_KP;
  cfg.ki = BMI088_HEATER_PID_KI;
  cfg.kd = BMI088_HEATER_PID_KD;
  cfg.maxout = BMI088_HEATER_PID_MAXOUT;
  cfg.integralLimit = BMI088_HEATER_PID_I_LIMIT;

  cfg.mode = PID_POSITION;
  cfg.improve = PID_IMPROVE_NONE;

  cfg.deadBand = 0.0f;
  cfg.coefA = 0.0f;
  cfg.coefB = 0.0f;
  cfg.derivative_LPF_RC = 0.0f;
  cfg.output_LPF_RC = 0.0f;

  s_heater.initialized = true;
  s_heater.enable = false;
  s_heater.preheat_finished = false;
  s_heater.now_temperature_c = 0.0f;
  s_heater.target_temperature_c = BMI088_HEATER_DEFAULT_TARGET_C;
  s_heater.supply_voltage_v = BMI088_HEATER_NOMINAL_VOLTAGE;
  s_heater.output_compare = 0U;

  PID_Init(&s_heater.pid, &cfg, BMI088_HEATER_PID_DT);
  PID_Reset(&s_heater.pid);

  HAL_TIM_PWM_Start(BMI088_HEATER_TIM_HANDLE, BMI088_HEATER_TIM_CHANNEL);
  __HAL_TIM_SET_COMPARE(BMI088_HEATER_TIM_HANDLE, BMI088_HEATER_TIM_CHANNEL,
                        0U);
}

void BMI088_Heater_Set_Enable(bool enable) {
  s_heater.enable = enable;

  if (!enable) {
    PID_Reset(&s_heater.pid);
    s_heater.preheat_finished = false;
    BMI088_Heater_Set_Compare(0U);
  }
}

void BMI088_Heater_Set_Target_Temperature_C(float target_temperature_c) {
  s_heater.target_temperature_c =
      BMI088_Heater_Clamp_Float(target_temperature_c, 20.0f, 80.0f);
}

void BMI088_Heater_TIM_128ms_PeriodElapsedCallback(void) {
  float power;
  float supply;
  float output;

  if (!s_heater.initialized) {
    return;
  }

  s_heater.now_temperature_c = BMI088_Get_Temperature_C();

  if (!s_heater.enable) {
    BMI088_Heater_Set_Compare(0U);
    return;
  }

  if (!isfinite(s_heater.now_temperature_c) ||
      s_heater.now_temperature_c > BMI088_HEATER_MAX_SAFE_TEMPERATURE_C) {
    PID_Reset(&s_heater.pid);
    s_heater.preheat_finished = false;
    BMI088_Heater_Set_Compare(0U);
    return;
  }

  s_heater.preheat_finished =
      (s_heater.now_temperature_c >= BMI088_HEATER_PREHEAT_BASE_TEMPERATURE_C);

  if (!s_heater.preheat_finished) {
    power = BMI088_HEATER_PREHEAT_POWER;
  } else {
    power = PID_Calculate(&s_heater.pid, s_heater.now_temperature_c,
                          s_heater.target_temperature_c);
  }

  supply = s_heater.supply_voltage_v;
  if (supply < 1.0f) {
    supply = BMI088_HEATER_NOMINAL_VOLTAGE;
  }

  output = power *
           (BMI088_HEATER_NOMINAL_VOLTAGE * BMI088_HEATER_NOMINAL_VOLTAGE) /
           (supply * supply);

  if (!isfinite(output)) {
    output = 0.0f;
  }

  output = BMI088_Heater_Clamp_Float(output, 0.0f,
                                     (float)BMI088_Heater_Get_Max_Compare());

  BMI088_Heater_Set_Compare((uint32_t)(output));
}
