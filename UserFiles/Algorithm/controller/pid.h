#ifndef PID_H
#define PID_H

#include <math.h>
#include <stdint.h>

/**
 * @brief PID 模式选择
 */
typedef enum {
  PID_POSITION = 0, // 位置式
  PID_DELTA = 1,    // 增量式
} pid_mode_e;

/**
 * @brief PID 优化环节使能标志位
 * @note  通过位或组合多个优化: PID_Integral_Limit | PID_DerivativeFilter
 */
typedef enum {
  PID_IMPROVE_NONE = 0x00,              // 纯净 PID
  PID_Integral_Limit = 0x01,            // 积分限幅（含抗饱和）
  PID_Derivative_On_Measurement = 0x02, // 微分先行（对反馈值微分而非误差）
  PID_Trapezoid_Integral = 0x04,        // 梯形积分
  PID_ChangingIntegrationRate = 0x08,   // 变速积分
  PID_DerivativeFilter = 0x10,          // 微分滤波
  PID_OutputFilter = 0x20,              // 输出滤波
  PID_DeadBand = 0x40,                  // 死区
} pid_improve_e;

/**
 * @brief PID 初始化配置结构体
 */
typedef struct {
  // 基本参数
  float kp;
  float ki;
  float kd;
  float maxout;
  float integralLimit;

  pid_mode_e mode;

  uint8_t improve; // 优化标志位组合

  // 优化参数（仅在对应 Improve 位启用时生效）
  float deadBand;          // 死区阈值
  float coefA;             // 变速积分系数 A
  float coefB;             // 变速积分系数 B
  float derivative_LPF_RC; // 微分滤波器时间常数 (RC = 1/omega_c)
  float output_LPF_RC;     // 输出滤波器时间常数
} pid_config_t;

/**
 * @brief PID 控制器实例
 */
typedef struct {
  // 参数
  float kp;
  float ki;
  float kd;
  float maxout;
  float integralLimit;

  pid_mode_e mode;
  uint8_t improve;

  float deadBand;
  float coefA;
  float coefB;
  float derivative_LPF_RC;
  float output_LPF_RC;

  // 状态变量
  float ref;
  float measure;
  float lastMeasure;
  float err;
  float lastErr;
  float lastLastErr;

  float pout;
  float iout; // 累计i
  float dout;
  float iTerm; // 增量i

  float output;
  float lastOutput;
  float lastDout;

  float dt;

  // 真实时间积分相关（基于 nowtime）
  uint32_t last_update_time; // 上次更新的时间戳（nowtime 值）
  uint8_t use_real_dt;       // 是否使用真实dt：1=启用，0=禁用（默认启用）
} PID_t;

extern void PID_Init(PID_t *pid, const pid_config_t *config, float default_dt);
extern float PID_Calculate(PID_t *pid, float measure, float ref);
extern void PID_Reset(PID_t *pid);
extern void PID_SetParams(PID_t *pid, float kp, float ki, float kd);

#endif // PID_H