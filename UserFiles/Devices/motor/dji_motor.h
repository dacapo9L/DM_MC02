#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

#include "bsp_can.h"
#include "pid.h"
#include "main.h"
#include <stdint.h>

// 一圈编码器刻度
#define ENCODER_NUM_PER_ROUND 8192;

// 扭矩电流常数
#define CURRENT_TO_TORQUE 0.741f
// 扭矩电流到输出刻度的转化系数
#define CURRENT_TO_OUT 16384.0f / 3.0f
// 最大输出刻度
#define OUT_MAXs 16384.0f;

enum Enum_Motor_DJI_Status {
  Motor_DJI_Status_DISABLE = 0,
  Motor_DJI_Status_ENABLE,
};

enum Enum_Motor_DJI_ID {
  Motor_DJI_ID_0x201 = 1,
  Motor_DJI_ID_0x202,
  Motor_DJI_ID_0x203,
  Motor_DJI_ID_0x204,
  Motor_DJI_ID_0x205,
  Motor_DJI_ID_0x206,
  Motor_DJI_ID_0x207,
  Motor_DJI_ID_0x208,
  Motor_DJI_ID_0x209,
  Motor_DJI_ID_0x20A,
  Motor_DJI_ID_0x20B,
};

enum Enum_Motor_DJI_Control_Method {
  Motor_DJI_Control_Method_TORQUE = 0,
  Motor_DJI_Control_Method_OMEGA,
  Motor_DJI_Control_Method_ANGLE,
};

enum Enum_Motor_DJI_Power_Limit_Status {
  Motor_DJI_Power_Limit_Status_DISABLE = 0,
  Motor_DJI_Power_Limit_Status_ENABLE,
};

enum Enum_Motor_DJI_Model {
  Motor_DJI_Model_GM6020 = 0,
  Motor_DJI_Model_C620,
  Motor_DJI_Model_C610,
};

#define DJI_MOTOR_ID_COUNT 11U

typedef struct Struct_Motor_DJI_CAN_Rx_Data {
  uint16_t encoder_reverse;
  int16_t speed_rpm;
  int16_t current_reverse;
  uint8_t temperature;
  uint8_t reserved;
} __attribute__((packed)) Struct_Motor_DJI_CAN_Rx_Data;

typedef struct Struct_Motor_DJI_Rx_Data {
  float now_angle;
  float now_omega;
  float now_torque;
  float now_temperature;
  float now_power;
  uint16_t now_encoder;
  int16_t now_current;
  int16_t now_speed_rpm;
  uint32_t pre_encoder;
  int32_t total_encoder;
  int32_t total_round;
} Struct_Motor_DJI_Rx_Data;

typedef struct Struct_DJI_Motor {
  Struct_CAN_Manage_Object *CAN_Manage_Object;
  enum Enum_Motor_DJI_ID CAN_Rx_ID;
  uint8_t *tx_data;

  int32_t encoder_offset;
  float nearest_angle;

  enum Enum_Motor_DJI_Status Motor_DJI_Status;
  enum Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method;
  enum Enum_Motor_DJI_Model Motor_DJI_Model;

  struct Struct_Motor_DJI_Rx_Data Rx_Data;

  float target_angle;
  float target_omega;
  float target_torque;
  float feedforward_omega;
  float feedforward_torque;

  uint32_t flag;
  uint32_t pre_flag;
  float out;
} Struct_DJI_Motor;

extern Struct_Motor_DJI_CAN_Rx_Data g_dji_feedback[DJI_MOTOR_ID_COUNT];
extern Struct_DJI_Motor *dji_motor_can1map[DJI_MOTOR_ID_COUNT];
extern Struct_DJI_Motor *dji_motor_can2map[DJI_MOTOR_ID_COUNT];
extern Struct_DJI_Motor *dji_motor_can3map[DJI_MOTOR_ID_COUNT];

uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan,
                          enum Enum_Motor_DJI_ID can_id);

void DJI_Motor_Init(Struct_DJI_Motor *motor, const FDCAN_HandleTypeDef *hcan,
                    enum Enum_Motor_DJI_ID can_id,
                    enum Enum_Motor_DJI_Control_Method control_method,
                    enum Enum_Motor_DJI_Model model, int32_t encoder_offset,
                    float nearest_angle);

void DJI_Motor_CAN_RxCpltCallback(Struct_DJI_Motor *motor);
void DJI_Motor_TIM_100ms_Alive_PeriodElapsedCallback(Struct_DJI_Motor *motor);
void DJI_Motor_TIM_Calculate_PeriodElapsedCallback(Struct_DJI_Motor *motor);
void DJI_Motor_Data_Process(Struct_DJI_Motor *motor);
void DJI_Motor_PID_Calculate(Struct_DJI_Motor *motor);

#endif
