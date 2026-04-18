#include "zdt_steppermotor.h"
#include "bsp_can.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

Struct_ZDT_StepperMotor *zdt_stepper_can1map[4] = {NULL};
Struct_ZDT_StepperMotor *zdt_stepper_can2map[4] = {NULL};
Struct_ZDT_StepperMotor *zdt_stepper_can3map[4] = {NULL};

typedef void (*ZDT_StepperMotor_Map_Iterator)(Struct_ZDT_StepperMotor *motor);

static bool zdt_can_id_to_map_index(enum Enum_Motor_ZDT_ID can_id,
                                    uint8_t *map_index) {
  if (map_index == NULL) {
    return false;
  }

  switch (can_id) {
  case Motor_ZDT_ID_0x001:
    *map_index = 0U;
    return true;
  case Motor_ZDT_ID_0x003:
    *map_index = 1U;
    return true;
  case Motor_ZDT_ID_0x004:
    *map_index = 2U;
    return true;
  case Motor_ZDT_ID_0x005:
    *map_index = 3U;
    return true;
  default:
    return false;
  }
}

static void for_each_registered_motor(ZDT_StepperMotor_Map_Iterator callback) {
  uint8_t i;

  if (callback == NULL) {
    return;
  }

  for (i = 0U; i < 4; i++) {
    if (zdt_stepper_can1map[i] != NULL) {
      callback(zdt_stepper_can1map[i]);
    }
    if (zdt_stepper_can2map[i] != NULL) {
      callback(zdt_stepper_can2map[i]);
    }
    if (zdt_stepper_can3map[i] != NULL) {
      callback(zdt_stepper_can3map[i]);
    }
  }
}

static void allocate_tx_data(const FDCAN_HandleTypeDef *hcan,
                             enum Enum_Motor_ZDT_ID can_id,
                             uint8_t *tx_data_ptr[2]) {
  tx_data_ptr[0] = NULL;
  tx_data_ptr[1] = NULL;

  if (hcan == &hfdcan1) {
    switch (can_id) {
    case Motor_ZDT_ID_0x001:
      tx_data_ptr[0] = &(CAN1_0x100_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN1_0x101_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x003:
      tx_data_ptr[0] = &(CAN1_0x300_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN1_0x301_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x004:
      tx_data_ptr[0] = &(CAN1_0x400_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN1_0x401_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x005:
      tx_data_ptr[0] = &(CAN1_0x500_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN1_0x501_Tx_Data[0]);
      break;
    default:
      break;
    }
  } else if (hcan == &hfdcan2) {
    switch (can_id) {
    case Motor_ZDT_ID_0x001:
      tx_data_ptr[0] = &(CAN2_0x100_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN2_0x101_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x003:
      tx_data_ptr[0] = &(CAN2_0x300_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN2_0x301_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x004:
      tx_data_ptr[0] = &(CAN2_0x400_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN2_0x401_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x005:
      tx_data_ptr[0] = &(CAN2_0x500_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN2_0x501_Tx_Data[0]);
      break;
    default:
      break;
    }
  } else if (hcan == &hfdcan3) {
    switch (can_id) {
    case Motor_ZDT_ID_0x001:
      tx_data_ptr[0] = &(CAN3_0x100_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN3_0x101_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x003:
      tx_data_ptr[0] = &(CAN3_0x300_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN3_0x301_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x004:
      tx_data_ptr[0] = &(CAN3_0x400_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN3_0x401_Tx_Data[0]);
      break;
    case Motor_ZDT_ID_0x005:
      tx_data_ptr[0] = &(CAN3_0x500_Tx_Data[0]);
      tx_data_ptr[1] = &(CAN3_0x501_Tx_Data[0]);
      break;
    default:
      break;
    }
  }
}

static void unregister_motor_from_map(Struct_ZDT_StepperMotor *motor) {
  uint8_t i;

  for (i = 0U; i < 4; i++) {
    if (zdt_stepper_can1map[i] == motor) {
      zdt_stepper_can1map[i] = NULL;
    }
    if (zdt_stepper_can2map[i] == motor) {
      zdt_stepper_can2map[i] = NULL;
    }
    if (zdt_stepper_can3map[i] == motor) {
      zdt_stepper_can3map[i] = NULL;
    }
  }
}

void ZDT_Stepper_Init(Struct_ZDT_StepperMotor *motor,
                      const FDCAN_HandleTypeDef *hcan,
                      enum Enum_Motor_ZDT_ID can_id) {
  uint8_t map_index;

  if (motor == NULL || hcan == NULL) {
    return;
  }

  if (!zdt_can_id_to_map_index(can_id, &map_index)) {
    return;
  }

  unregister_motor_from_map(motor);
  memset(motor, 0, sizeof(*motor));

  if (hcan->Instance == FDCAN1) {
    motor->CAN_Manage_Object = &CAN1_Manage_Object;
    zdt_stepper_can1map[map_index] = motor;
  } else if (hcan->Instance == FDCAN2) {
    motor->CAN_Manage_Object = &CAN2_Manage_Object;
    zdt_stepper_can2map[map_index] = motor;
  } else if (hcan->Instance == FDCAN3) {
    motor->CAN_Manage_Object = &CAN3_Manage_Object;
    zdt_stepper_can3map[map_index] = motor;
  } else {
    motor->CAN_Manage_Object = NULL;
  }

  motor->CAN_Rx_ID = can_id;
  allocate_tx_data(hcan, can_id, motor->tx_data);
}

void ZDT_Stepper_Control(Struct_ZDT_StepperMotor *motor, uint8_t func,
                         bool snf) {
  if (motor == NULL || motor->tx_data[0] == NULL) {
    return;
  }

  switch (func) {
  case EN:
    if (motor == NULL) {
      return;
    }

    motor->tx_data[0][0] = 0xF3;
    motor->tx_data[0][1] = 0xAB;
    motor->tx_data[0][2] = (uint8_t)(motor->Tx_Params.state ? 1U : 0U);
    motor->tx_data[0][3] = (uint8_t)(snf ? 1U : 0U);
    motor->tx_data[0][4] = 0x6B;
    break;
  case VEL:
    if (motor == NULL) {
      return;
    }

    motor->tx_data[0][0] = 0xF6;
    motor->tx_data[0][1] = motor->Tx_Params.dir;
    motor->tx_data[0][2] = (uint8_t)(motor->Tx_Params.vel >> 8U);
    motor->tx_data[0][3] = (uint8_t)(motor->Tx_Params.vel >> 0U);
    motor->tx_data[0][4] = motor->Tx_Params.acc;
    motor->tx_data[0][5] = (uint8_t)(snf ? 1U : 0U);
    motor->tx_data[0][6] = 0x6B;
    break;
  case POS:
    if (motor == NULL) {
      return;
    }

    motor->tx_data[0][0] = 0xFD;
    motor->tx_data[0][1] = motor->Tx_Params.dir;
    motor->tx_data[0][2] = (uint8_t)(motor->Tx_Params.vel >> 8U);
    motor->tx_data[0][3] = (uint8_t)(motor->Tx_Params.vel >> 0U);
    motor->tx_data[0][4] = motor->Tx_Params.acc;
    motor->tx_data[0][5] = (uint8_t)(motor->Tx_Params.clk >> 24U);
    motor->tx_data[0][6] = (uint8_t)(motor->Tx_Params.clk >> 16U);
    motor->tx_data[0][7] = (uint8_t)(motor->Tx_Params.clk >> 8U);

    motor->tx_data[1][0] = 0xFD;
    motor->tx_data[1][1] = (uint8_t)(motor->Tx_Params.clk >> 0U);
    motor->tx_data[1][2] = (uint8_t)(motor->Tx_Params.raf ? 1U : 0U);
    motor->tx_data[1][3] = (uint8_t)(snf ? 1U : 0U);
    motor->tx_data[1][4] = 0x6B;
    break;
  case STOP_NOW:
    motor->tx_data[0][0] = 0xFE;
    motor->tx_data[0][1] = 0x98;
    motor->tx_data[0][2] = (uint8_t)(snf ? 1U : 0U);
    motor->tx_data[0][3] = 0x6B;
    break;
  case READ_SYS_PARAMS:
    if (motor == NULL) {
      return;
    }

    if (motor->Rx_Params != ZDT_SYS_VEL && motor->Rx_Params != ZDT_SYS_CPOS) {
      return;
    }

    motor->tx_data[0][0] = (uint8_t)motor->Rx_Params;
    motor->tx_data[0][1] = 0x6B;
    break;
  default:
    break;
  }
}

void ZDT_Stepper_CAN_RxCpltCallback(Struct_ZDT_StepperMotor *motor) {
  const FDCAN_RxHeaderTypeDef *hdr;

  if (motor == NULL || motor->CAN_Manage_Object == NULL) {
    return;
  }

  hdr = &motor->CAN_Manage_Object->Rx_Header;

  if (hdr->IdType != FDCAN_EXTENDED_ID) {
    return;
  }

  motor->flag += 1U;
  motor->online = true;
}

void ZDT_Stepper_DataProcess(Struct_ZDT_StepperMotor *motor) {
  if (motor == NULL || motor->CAN_Manage_Object == NULL) {
    return;
  }
}

void ZDT_Stepper_TIM_100ms_Alive_PeriodElapsedCallback(
    Struct_ZDT_StepperMotor *motor) {

  if (motor == NULL) {
    for_each_registered_motor(
        ZDT_Stepper_TIM_100ms_Alive_PeriodElapsedCallback);
    return;
  }

  if (motor->flag == motor->pre_flag) {
    motor->online = false;
  } else {
    motor->online = true;
  }

  motor->pre_flag = motor->flag;
}

void ZDT_Stepper_TIM_Calculate_PeriodElapsedCallback(
    Struct_ZDT_StepperMotor *motor) {
  if (motor == NULL) {
    for_each_registered_motor(ZDT_Stepper_TIM_Calculate_PeriodElapsedCallback);
    return;
  }
}
