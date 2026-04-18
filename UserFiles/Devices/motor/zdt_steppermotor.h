#ifndef ZDT_STEPPERMOTOR_H
#define ZDT_STEPPERMOTOR_H

#include "bsp_can.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

enum Enum_Motor_ZDT_ID {
  Motor_ZDT_ID_0x001 = 0x001,
  Motor_ZDT_ID_0x003 = 0x003,
  Motor_ZDT_ID_0x004 = 0x004,
  Motor_ZDT_ID_0x005 = 0x005,
};

typedef struct {
  bool state;
  uint8_t dir;
  uint16_t vel;
  uint8_t acc;
  uint32_t clk;
  bool raf;
} Struct_ZDT_StepperMotor_Tx_Params;

typedef enum {
  EN = 0,
  VEL,
  POS,
  STOP_NOW,
  READ_SYS_PARAMS
} Struct_ZDT_StepperMotor_Function;

typedef enum {
  ZDT_SYS_VEL = 0x35,
  ZDT_SYS_CPOS = 0x36,
} Struct_ZDT_StepperMotor_Rx_Params;

typedef struct {
  Struct_CAN_Manage_Object *CAN_Manage_Object;
  Struct_ZDT_StepperMotor_Tx_Params Tx_Params;
  Struct_ZDT_StepperMotor_Rx_Params Rx_Params;
  Struct_ZDT_StepperMotor_Function *Control_Functions;
  enum Enum_Motor_ZDT_ID CAN_Rx_ID;
  uint8_t *tx_data[2];

  uint32_t flag;
  uint32_t pre_flag;
  bool online;

  int16_t now_vel_rpm;
  int32_t now_pos_raw;
} Struct_ZDT_StepperMotor;

extern Struct_ZDT_StepperMotor *zdt_stepper_can1map[4];
extern Struct_ZDT_StepperMotor *zdt_stepper_can2map[4];
extern Struct_ZDT_StepperMotor *zdt_stepper_can3map[4];

void ZDT_Stepper_Init(Struct_ZDT_StepperMotor *motor,
                      const FDCAN_HandleTypeDef *hcan,
                      enum Enum_Motor_ZDT_ID can_id);
void ZDT_Stepper_Control(Struct_ZDT_StepperMotor *motor, uint8_t func,
                         bool snf);
void ZDT_Stepper_CAN_RxCpltCallback(Struct_ZDT_StepperMotor *motor);
void ZDT_Stepper_DataProcess(Struct_ZDT_StepperMotor *motor);
void ZDT_Stepper_TIM_100ms_Alive_PeriodElapsedCallback(
    Struct_ZDT_StepperMotor *motor);

#endif
