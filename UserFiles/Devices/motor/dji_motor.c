#include "dji_motor.h"
#include <stddef.h>
#include <string.h>

Struct_DJI_Motor *dji_motor_can1map[DJI_MOTOR_ID_COUNT] = {NULL};
Struct_DJI_Motor *dji_motor_can2map[DJI_MOTOR_ID_COUNT] = {NULL};
Struct_DJI_Motor *dji_motor_can3map[DJI_MOTOR_ID_COUNT] = {NULL};

typedef void (*DJI_Motor_Map_Iterator)(Struct_DJI_Motor *motor);

// 遍历已经注册的电机, 对每个电机调用callback
static void for_each_registered_motor(DJI_Motor_Map_Iterator callback) {
  uint8_t i;

  if (callback == NULL) {
    return;
  }

  for (i = 0U; i < DJI_MOTOR_ID_COUNT; i++) {
    if (dji_motor_can1map[i] != NULL) {
      callback(dji_motor_can1map[i]);
    }
    if (dji_motor_can2map[i] != NULL) {
      callback(dji_motor_can2map[i]);
    }
    if (dji_motor_can3map[i] != NULL) {
      callback(dji_motor_can3map[i]);
    }
  }
}

static uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan,
                          enum Enum_Motor_DJI_ID can_id) {
  uint8_t *tx_data_ptr = NULL;

  if (hcan == &hfdcan1) {
    switch (can_id) {
    case Motor_DJI_ID_0x201:
      tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x202:
      tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x203:
      tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x204:
      tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x205:
      tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x206:
      tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x207:
      tx_data_ptr = &(CAN1_0x1fe_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x208:
      tx_data_ptr = &(CAN1_0x1fe_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x209:
      tx_data_ptr = &(CAN1_0x2fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x20A:
      tx_data_ptr = &(CAN1_0x2fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x20B:
      tx_data_ptr = &(CAN1_0x2fe_Tx_Data[4]);
      break;
    default:
      break;
    }
  } else if (hcan == &hfdcan2) {
    switch (can_id) {
    case Motor_DJI_ID_0x201:
      tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x202:
      tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x203:
      tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x204:
      tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x205:
      tx_data_ptr = &(CAN2_0x1fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x206:
      tx_data_ptr = &(CAN2_0x1fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x207:
      tx_data_ptr = &(CAN2_0x1fe_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x208:
      tx_data_ptr = &(CAN2_0x1fe_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x209:
      tx_data_ptr = &(CAN2_0x2fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x20A:
      tx_data_ptr = &(CAN2_0x2fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x20B:
      tx_data_ptr = &(CAN2_0x2fe_Tx_Data[4]);
      break;
    default:
      break;
    }
  } else if (hcan == &hfdcan3) {
    switch (can_id) {
    case Motor_DJI_ID_0x201:
      tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x202:
      tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x203:
      tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x204:
      tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x205:
      tx_data_ptr = &(CAN3_0x1fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x206:
      tx_data_ptr = &(CAN3_0x1fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x207:
      tx_data_ptr = &(CAN3_0x1fe_Tx_Data[4]);
      break;
    case Motor_DJI_ID_0x208:
      tx_data_ptr = &(CAN3_0x1fe_Tx_Data[6]);
      break;
    case Motor_DJI_ID_0x209:
      tx_data_ptr = &(CAN3_0x2fe_Tx_Data[0]);
      break;
    case Motor_DJI_ID_0x20A:
      tx_data_ptr = &(CAN3_0x2fe_Tx_Data[2]);
      break;
    case Motor_DJI_ID_0x20B:
      tx_data_ptr = &(CAN3_0x2fe_Tx_Data[4]);
      break;
    default:
      break;
    }
  }

  return tx_data_ptr;
}

static void unregister_motor_from_map(Struct_DJI_Motor *motor) {
  uint8_t i;

  for (i = 0U; i < DJI_MOTOR_ID_COUNT; i++) {
    if (dji_motor_can1map[i] == motor) {
      dji_motor_can1map[i] = NULL;
    }
    if (dji_motor_can2map[i] == motor) {
      dji_motor_can2map[i] = NULL;
    }
    if (dji_motor_can3map[i] == motor) {
      dji_motor_can3map[i] = NULL;
    }
  }
}

void DJI_Motor_Init(Struct_DJI_Motor *motor, const FDCAN_HandleTypeDef *hcan,
                    enum Enum_Motor_DJI_ID can_id,
                    enum Enum_Motor_DJI_Control_Method control_method,
                    enum Enum_Motor_DJI_Model model, int32_t encoder_offset,
                    float nearest_angle) {
  if (motor == NULL || hcan == NULL) {
    return;
  }

  unregister_motor_from_map(motor);
  memset(motor, 0, sizeof(*motor));

  if (hcan->Instance == FDCAN1) {
    motor->CAN_Manage_Object = &CAN1_Manage_Object;
    uint8_t map_index = (uint8_t)can_id - 1U;
    if (map_index < DJI_MOTOR_ID_COUNT) {
      dji_motor_can1map[map_index] = motor;
    }
  } else if (hcan->Instance == FDCAN2) {
    motor->CAN_Manage_Object = &CAN2_Manage_Object;
    uint8_t map_index = (uint8_t)can_id - 1U;
    if (map_index < DJI_MOTOR_ID_COUNT) {
      dji_motor_can2map[map_index] = motor;
    }
  } else if (hcan->Instance == FDCAN3) {
    motor->CAN_Manage_Object = &CAN3_Manage_Object;
    uint8_t map_index = (uint8_t)can_id - 1U;
    if (map_index < DJI_MOTOR_ID_COUNT) {
      dji_motor_can3map[map_index] = motor;
    }
  } else {
    motor->CAN_Manage_Object = NULL;
  }

  motor->CAN_Rx_ID = can_id;
  motor->Motor_DJI_Control_Method = control_method;
  motor->Motor_DJI_Model = model;
  motor->encoder_offset = encoder_offset;
  motor->nearest_angle = nearest_angle;
  motor->tx_data = allocate_tx_data(hcan, can_id);
  motor->Motor_DJI_Status = Motor_DJI_Status_DISABLE;
}

void DJI_Motor_CAN_RxCpltCallback(Struct_DJI_Motor *motor) {
  if (motor == NULL) {
    return;
  }

  // 滑动窗口,判断电机是否在线
  motor->flag += 1;

  DJI_Motor_Data_Process(motor);
}

void DJI_Motor_Data_Process(Struct_DJI_Motor *motor) {
  if (motor == NULL || motor->CAN_Manage_Object == NULL) {
    return;
  }

  // 统一的解包方法
  const uint8_t *rx_buffer = motor->CAN_Manage_Object->Rx_Buffer;
  motor->Rx_Data.now_encoder =
      (uint16_t)(((uint16_t)rx_buffer[0] << 8U) | (uint16_t)rx_buffer[1]);
  motor->Rx_Data.now_speed_rpm =
      (int16_t)(((uint16_t)rx_buffer[2] << 8U) | (uint16_t)rx_buffer[3]);
  motor->Rx_Data.now_current =
      (int16_t)(((uint16_t)rx_buffer[4] << 8U) | (uint16_t)rx_buffer[5]);
  motor->Rx_Data.now_temperature = (float)rx_buffer[6];

  if (motor->Motor_DJI_Model == Motor_DJI_Model_GM6020) {

  } else if (motor->Motor_DJI_Model == Motor_DJI_Model_C620) {

  } else if (motor->Motor_DJI_Model == Motor_DJI_Model_C610) {
  }
}

void DJI_Motor_PID_Calculate(Struct_DJI_Motor *motor) {
  if (motor == NULL) {
    return;
  }

  // motor->tx_data[0] = (int16_t)(0 >> 8);
  //  motor->tx_data[1] = (int16_t)(0 & 0xFF);
  if (motor->Motor_DJI_Model == Motor_DJI_Model_GM6020) {
    switch (motor->Motor_DJI_Control_Method) {
    case Motor_DJI_Control_Method_TORQUE:

      break;
    case Motor_DJI_Control_Method_OMEGA:

      break;
    case Motor_DJI_Control_Method_ANGLE:

      break;
    default:
      break;
    }
  } else if (motor->Motor_DJI_Model == Motor_DJI_Model_C620) {
    switch (motor->Motor_DJI_Control_Method) {
    case Motor_DJI_Control_Method_TORQUE:

      break;
    case Motor_DJI_Control_Method_OMEGA:

      break;
    case Motor_DJI_Control_Method_ANGLE:

      break;
    default:
      break;
    }
  } else if (motor->Motor_DJI_Model == Motor_DJI_Model_C610) {
    switch (motor->Motor_DJI_Control_Method) {
    case Motor_DJI_Control_Method_TORQUE:

      break;
    case Motor_DJI_Control_Method_OMEGA:

      break;
    case Motor_DJI_Control_Method_ANGLE:

      break;
    default:
      break;
    }
  }
}

void DJI_Motor_TIM_100ms_Alive_PeriodElapsedCallback(Struct_DJI_Motor *motor) {
  if (motor == NULL) {
    for_each_registered_motor(DJI_Motor_TIM_100ms_Alive_PeriodElapsedCallback);
    return;
  }

  if (motor->flag == motor->pre_flag) {
    motor->Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // PID_Angle.Set_Integral_Error(0.0f);
    // PID_Omega.Set_Integral_Error(0.0f);
  } else {

    motor->Motor_DJI_Status = Motor_DJI_Status_ENABLE;
  }
  motor->pre_flag = motor->flag;
}

void DJI_Motor_TIM_Calculate_PeriodElapsedCallback(Struct_DJI_Motor *motor) {
  if (motor == NULL) {
    for_each_registered_motor(DJI_Motor_TIM_Calculate_PeriodElapsedCallback);
    return;
  }

  DJI_Motor_PID_Calculate(motor);

  motor->feedforward_omega = 0.0f;
  motor->feedforward_torque = 0.0f;
}
