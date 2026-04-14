#include "pid.h"
#include "bsp_dwt.h"
#include <string.h>

/*========真实DT========*/

/**
 * @brief 获取真实的时间间隔
 * @param default_dt 默认时间间隔（秒），用于 DWT 异常时回退
 * @return 实际时间间隔（秒）
 */
static float Get_Real_DT(float default_dt) {
  uint32_t dwt_cnt = 0;
  float dt = DWT_Get_Delta_T(&dwt_cnt);
  if (dt <= 0.0f || dt > 1.0f) {
    // 如果dt无效，返回默认的dt
    return default_dt;
  }
  return dt;
}

/*========优化环节========*/

/**
 * @brief 梯形积分
 * @note  使用梯形面积公式: (上底 + 下底) * 高 / 2
 */
static void Trapezoid_Integral(PID_t *pid) {
  pid->iTerm = pid->ki * ((pid->err + pid->lastErr) / 2.0f) * pid->dt;
}

/**
 * @brief 变速积分
 * @note  误差小时积分作用强，误差大时积分作用弱或为零
 *        ITerm *= (A - |err| + B) / A, when B < |err| < A + B
 */
static void Changing_Integration_Rate(PID_t *pid) {
  if (pid->err * pid->iout > 0) {
    // 积分呈累积趋势
    float abs_err = fabsf(pid->err);
    if (abs_err <= pid->coefB) {
      // 误差很小，全积分
      return;
    }
    if (abs_err <= (pid->coefA + pid->coefB)) {
      // 误差中等，部分积分
      pid->iTerm *= (pid->coefA - abs_err + pid->coefB) / pid->coefA;
    } else {
      // 误差过大，不积分
      pid->iTerm = 0;
    }
  }
}

/**
 * @brief 积分限幅
 * @note  防止积分饱和，同时实现抗饱和（当输出饱和且积分累积时停止积分）
 */
static void Integral_Limit(PID_t *pid) {
  float temp_Iout = pid->iout + pid->iTerm;
  float temp_Output = pid->pout + pid->iout + pid->dout;

  // 抗饱和：输出饱和且积分仍在累积时，停止积分
  if (fabsf(temp_Output) > pid->maxout) {
    if (pid->err * pid->iout > 0) {
      pid->iTerm = 0;
    }
  }

  // 积分限幅
  if (temp_Iout > pid->integralLimit) {
    pid->iTerm = 0;
    pid->iout = pid->integralLimit;
  } else if (temp_Iout < -pid->integralLimit) {
    pid->iTerm = 0;
    pid->iout = -pid->integralLimit;
  }
}

/**
 * @brief 微分先行
 * @note  对测量值微分而非误差，避免目标值突变时产生微分尖峰
 */
static void Derivative_On_Measurement(PID_t *pid) {
  pid->dout = pid->kd * (pid->lastMeasure - pid->measure) / pid->dt;
}

/**
 * @brief 微分滤波
 * @note  一阶低通滤波，滤除高频噪声
 */
static void Derivative_Filter(PID_t *pid) {
  float alpha = pid->dt / (pid->derivative_LPF_RC + pid->dt);
  pid->dout = alpha * pid->dout + (1.0f - alpha) * pid->lastDout;
}

/**
 * @brief 输出滤波
 * @note  一阶低通滤波，平滑输出
 */
static void Output_Filter(PID_t *pid) {
  float alpha = pid->dt / (pid->output_LPF_RC + pid->dt);
  pid->output = alpha * pid->output + (1.0f - alpha) * pid->lastOutput;
}

/**
 * @brief 输出限幅
 */
static void Output_Limit(PID_t *pid) {
  if (pid->output > pid->maxout) {
    pid->output = pid->maxout;
  } else if (pid->output < -pid->maxout) {
    pid->output = -pid->maxout;
  }
}

/*========实现环节========*/

/**
 * @brief 位置式
 */
static float PID_Position_Calculate(PID_t *pid) {

  pid->pout = pid->kp * pid->err;

  pid->iTerm = pid->ki * pid->err * pid->dt;

  if (pid->improve & PID_Trapezoid_Integral) {
    Trapezoid_Integral(pid);
  }

  if (pid->improve & PID_ChangingIntegrationRate) {
    Changing_Integration_Rate(pid);
  }

  if (pid->improve & PID_Derivative_On_Measurement) {
    Derivative_On_Measurement(pid);
  } else {
    pid->dout = pid->kd * (pid->err - pid->lastErr) / pid->dt;
  }

  if (pid->improve & PID_DerivativeFilter) {
    Derivative_Filter(pid);
  }

  if (pid->improve & PID_Integral_Limit) {
    Integral_Limit(pid);
  }

  pid->iout += pid->iTerm;

  // 简单积分限幅（如果没有启用高级积分限幅）
  if (!(pid->improve & PID_Integral_Limit)) {
    if (pid->iout > pid->integralLimit) {
      pid->iout = pid->integralLimit;
    } else if (pid->iout < -pid->integralLimit) {
      pid->iout = -pid->integralLimit;
    }
  }

  pid->output = pid->pout + pid->iout + pid->dout;

  return pid->output;
}

/**
 * @brief 增量式
 */
static float PID_Delta_Calculate(PID_t *pid) {

  pid->pout = pid->kp * (pid->err - pid->lastErr);

  pid->iTerm = pid->ki * pid->err * pid->dt;

  if (pid->improve & PID_ChangingIntegrationRate) {
    Changing_Integration_Rate(pid);
  }

  pid->iout = pid->iTerm; // 增量式中Iout就是本次的I增量

  if (pid->improve & PID_Derivative_On_Measurement) {
    pid->dout = pid->kd *
                (2.0f * pid->lastMeasure - pid->measure - pid->lastLastErr) /
                pid->dt;
  } else {
    pid->dout =
        pid->kd * (pid->err - 2.0f * pid->lastErr + pid->lastLastErr) / pid->dt;
  }

  if (pid->improve & PID_DerivativeFilter) {
    Derivative_Filter(pid);
  }

  float delta_output = pid->pout + pid->iout + pid->dout;

  pid->output = pid->lastOutput + delta_output;

  return pid->output;
}

/*========对外接口========*/

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 结构体指针
 * @param config PID 配置参数
 * @param default_dt 默认时间间隔（秒）
 */
void PID_Init(PID_t *pid, const pid_config_t *config, float default_dt) {
  memset(pid, 0, sizeof(PID_t));

  pid->kp = config->kp;
  pid->ki = config->ki;
  pid->kd = config->kd;
  pid->maxout = config->maxout;
  pid->integralLimit = config->integralLimit;

  pid->mode = config->mode;
  pid->improve = config->improve;

  pid->deadBand = config->deadBand;
  pid->coefA = config->coefA;
  pid->coefB = config->coefB;
  pid->derivative_LPF_RC = config->derivative_LPF_RC;
  pid->output_LPF_RC = config->output_LPF_RC;

  pid->dt = Get_Real_DT(default_dt);
}

/**
 * @brief PID 计算
 * @param pid PID 结构体指针
 * @param measure 当前测量值
 * @param ref 目标参考值
 * @return PID 输出值
 */
float PID_Calculate(PID_t *pid, float measure, float ref) {
  pid->measure = measure;
  pid->ref = ref;
  pid->err = ref - measure;

  if ((pid->improve & PID_DeadBand) && fabsf(pid->err) < pid->deadBand) {
    pid->output = (pid->mode == PID_DELTA) ? pid->lastOutput : 0;
    pid->iTerm = 0;
    // 更新历史值
    pid->lastLastErr = pid->lastErr;
    pid->lastMeasure = measure;
    pid->lastErr = pid->err;
    return pid->output;
  }

  if (pid->mode == PID_POSITION) {
    PID_Position_Calculate(pid);
  } else {
    PID_Delta_Calculate(pid);
  }

  if (pid->improve & PID_OutputFilter) {
    Output_Filter(pid);
  }

  Output_Limit(pid);

  // 保存历史
  pid->lastLastErr = pid->lastErr;
  pid->lastMeasure = measure;
  pid->lastOutput = pid->output;
  pid->lastDout = pid->dout;
  pid->lastErr = pid->err;

  return pid->output;
}

/**
 * @brief 重置 PID 控制器状态
 * @param pid PID 结构体指针
 */
void PID_Reset(PID_t *pid) {
  pid->err = 0;
  pid->lastErr = 0;
  pid->lastLastErr = 0;
  pid->iout = 0;
  pid->iTerm = 0;
  pid->dout = 0;
  pid->lastDout = 0;
  pid->pout = 0;
  pid->output = 0;
  pid->lastOutput = 0;
  pid->lastMeasure = 0;
}

/**
 * @brief 设置 PID 参数
 * @param pid PID 结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_SetParams(PID_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}