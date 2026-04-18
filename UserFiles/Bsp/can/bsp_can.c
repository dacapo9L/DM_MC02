#include "bsp_can.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "zdt_steppermotor.h"
#include <stdbool.h>

Struct_CAN_Manage_Object CAN1_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN2_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN3_Manage_Object = {NULL};

// CAN通信发送缓冲区

// 电机共享�?

// CAN1
uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

// ZDT
uint8_t CAN1_0x100_Tx_Data[8];
uint8_t CAN1_0x101_Tx_Data[8];
uint8_t CAN1_0x300_Tx_Data[8];
uint8_t CAN1_0x301_Tx_Data[8];
uint8_t CAN1_0x400_Tx_Data[8];
uint8_t CAN1_0x401_Tx_Data[8];
uint8_t CAN1_0x500_Tx_Data[8];
uint8_t CAN1_0x501_Tx_Data[8];

// CAN2
uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

// ZDT
uint8_t CAN2_0x100_Tx_Data[8];
uint8_t CAN2_0x101_Tx_Data[8];
uint8_t CAN2_0x300_Tx_Data[8];
uint8_t CAN2_0x301_Tx_Data[8];
uint8_t CAN2_0x400_Tx_Data[8];
uint8_t CAN2_0x401_Tx_Data[8];
uint8_t CAN2_0x500_Tx_Data[8];
uint8_t CAN2_0x501_Tx_Data[8];

// CAN3
uint8_t CAN3_0x1fe_Tx_Data[8];
uint8_t CAN3_0x1ff_Tx_Data[8];
uint8_t CAN3_0x200_Tx_Data[8];
uint8_t CAN3_0x2fe_Tx_Data[8];
uint8_t CAN3_0x2ff_Tx_Data[8];
uint8_t CAN3_0x3fe_Tx_Data[8];
uint8_t CAN3_0x4fe_Tx_Data[8];

// ZDT
uint8_t CAN3_0x100_Tx_Data[8];
uint8_t CAN3_0x101_Tx_Data[8];
uint8_t CAN3_0x300_Tx_Data[8];
uint8_t CAN3_0x301_Tx_Data[8];
uint8_t CAN3_0x400_Tx_Data[8];
uint8_t CAN3_0x401_Tx_Data[8];
uint8_t CAN3_0x500_Tx_Data[8];
uint8_t CAN3_0x501_Tx_Data[8];

/**
 * @brief 配置CAN的过滤器
 * 默认开了fifo0和fifo1的全通滤波器, 但由于fifo0和fifo1匹配规则一致,
 * 因此fifo1理论上不会被触发, 即使fifo0满 如若出现接收满的情况,
 * 可配置掩码选择性接收, 合理分担总线带宽 此外,
 * Cortex-M7内核的滤波器配置在每个CAN实例都是独立的, 且滤波器编号也都是独立的
 * 如, F4系列芯片的CAN1和CAN2的滤波器编号分别是0-13和14-27,
 * 但H7系列芯片的FDCAN1和FDCAN2的滤波器编号都可以从0开始
 *
 * @param hfdcan CAN编号
 */
void can_filter_mask_config(FDCAN_HandleTypeDef *hfdcan, bool extid) {
  FDCAN_FilterTypeDef can_filter_init_structure;

  // 配置fifo0全通滤波器
  can_filter_init_structure.IdType =
      extid ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  can_filter_init_structure.FilterIndex = 0;
  can_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
  can_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_init_structure.FilterID1 = 0x00000000;
  can_filter_init_structure.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(hfdcan, &can_filter_init_structure);

  // 全局滤波 直接拒绝不符合规则的标准数据帧、扩展数据帧、标准遥控帧
  // 扩展遥控帧
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 启动CAN中断与总线
  HAL_FDCAN_ActivateNotification(hfdcan,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE |
                                     FDCAN_IT_ARB_PROTOCOL_ERROR |
                                     FDCAN_IT_DATA_PROTOCOL_ERROR,
                                 0);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hfdcan CAN编号
 * @param extid ID类型
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, bool extid,
              CAN_Callback Callback_Function) {
  if (hfdcan->Instance == FDCAN1) {
    CAN1_Manage_Object.CAN_Handler = hfdcan;
    CAN1_Manage_Object.Callback_Function = Callback_Function;
  } else if (hfdcan->Instance == FDCAN2) {
    CAN2_Manage_Object.CAN_Handler = hfdcan;
    CAN2_Manage_Object.Callback_Function = Callback_Function;
  } else if (hfdcan->Instance == FDCAN3) {
    CAN3_Manage_Object.CAN_Handler = hfdcan;
    CAN3_Manage_Object.Callback_Function = Callback_Function;
  }

  can_filter_mask_config(hfdcan, extid);

  HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, bool exid,
                          uint8_t *Data, uint16_t Length) {
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = ID;
  tx_header.IdType = exid ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = Length;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_100us_CAN_PeriodElapsedCallback() {}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_CAN_PeriodElapsedCallback() {
  // DJI电机专属
  static int dji = 0;
  dji++;
  if (dji == 2) {
    dji = 0;

    // CAN_Transmit_Data(&hfdcan1, 0x1fe, false, CAN1_0x1fe_Tx_Data, 8);
  }

  // ZDT电机专属
  static int zdt = 0;
  zdt++;
  if (zdt == 10) {
    zdt = 0;
    static uint8_t rr = 0; // round-robin index
    static uint8_t pos_stage[4] = {0U};

    for (uint8_t k = 0; k < 4; k++) {
      uint8_t i = (rr + k) % 4;
      Struct_ZDT_StepperMotor *motor = zdt_stepper_can2map[i];
      uint16_t len0 = 0U;
      uint8_t func;
      uint32_t id0, id1;

      if (motor == NULL || motor->tx_data[0] == NULL) {
        pos_stage[i] = 0U;
        continue;
      }

      id0 = ((uint32_t)motor->CAN_Rx_ID << 8U) | 0U;
      id1 = id0 | 1U;

      if (pos_stage[i] != 0U) {
        if (motor->tx_data[0][0] != 0xFDU || motor->tx_data[1] == NULL) {
          pos_stage[i] = 0U;
          continue;
        }

        if (CAN_Transmit_Data(&hfdcan2, id1, true, motor->tx_data[1], 5U) ==
            HAL_OK) {
          pos_stage[i] = 0U;
          motor->tx_data[0][0] = 0U;
          motor->tx_data[1][0] = 0U;
        }

        rr = (i + 1U) % 4U;
        break;
      }

      func = motor->tx_data[0][0];
      switch (func) {
      case 0xF3:
        len0 = 5U;
        break;
      case 0xF6:
        len0 = 7U;
        break;
      case 0xFD:
        len0 = 8U;
        break;
      case 0xFE:
        len0 = 4U;
        break;
      case 0x35:
      case 0x36:
        len0 = 2U;
        break;
      default:
        break;
      }
      if (len0 == 0U) {
        continue;
      }

      if (func == 0xFDU) {
        if (motor->tx_data[1] == NULL) {
          continue;
        }

        if (CAN_Transmit_Data(&hfdcan2, id0, true, motor->tx_data[0], len0) ==
            HAL_OK) {
          pos_stage[i] = 1U;
        }

        rr = (i + 1U) % 4U;
        break;
      }

      if (CAN_Transmit_Data(&hfdcan2, id0, true, motor->tx_data[0], len0) ==
          HAL_OK) {
        motor->tx_data[0][0] = 0U;
      }

      rr = (i + 1U) % 4U;
      break;
    }
  }
}
/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hfdcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  // 判断程序初始化完
  if (0) {
    // 也得接收, 防止FIFO满了之后无法接收新消息, 但不处理
    if (hfdcan->Instance == FDCAN1) {
      while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &CAN1_Manage_Object.Rx_Header,
                                    CAN1_Manage_Object.Rx_Buffer) == HAL_OK) {
      }
    } else if (hfdcan->Instance == FDCAN2) {
      while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &CAN2_Manage_Object.Rx_Header,
                                    CAN2_Manage_Object.Rx_Buffer) == HAL_OK) {
      }
    } else if (hfdcan->Instance == FDCAN3) {
      while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &CAN3_Manage_Object.Rx_Header,
                                    CAN3_Manage_Object.Rx_Buffer) == HAL_OK) {
      }
    }
    return;
  }

  // 选择回调函数
  if (hfdcan->Instance == FDCAN1) {
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                  &CAN1_Manage_Object.Rx_Header,
                                  CAN1_Manage_Object.Rx_Buffer) == HAL_OK) {
      CAN1_Manage_Object.Rx_Timestamp = (uint64_t)DWT_Get_Timestamp();

      if (CAN1_Manage_Object.Callback_Function != NULL) {
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Header,
                                             CAN1_Manage_Object.Rx_Buffer);
      }
    }
  } else if (hfdcan->Instance == FDCAN2) {
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                  &CAN2_Manage_Object.Rx_Header,
                                  CAN2_Manage_Object.Rx_Buffer) == HAL_OK) {
      CAN2_Manage_Object.Rx_Timestamp = (uint64_t)DWT_Get_Timestamp();

      if (CAN2_Manage_Object.Callback_Function != NULL) {
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Header,
                                             CAN2_Manage_Object.Rx_Buffer);
      }
    }
  } else if (hfdcan->Instance == FDCAN3) {
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                  &CAN3_Manage_Object.Rx_Header,
                                  CAN3_Manage_Object.Rx_Buffer) == HAL_OK) {
      CAN3_Manage_Object.Rx_Timestamp = (uint64_t)DWT_Get_Timestamp();

      if (CAN3_Manage_Object.Callback_Function != NULL) {
        CAN3_Manage_Object.Callback_Function(&CAN3_Manage_Object.Rx_Header,
                                             CAN3_Manage_Object.Rx_Buffer);
      }
    }
  }
}

/**
 * @brief HAL库CAN错误中断
 *
 * @param hfdcan CAN编号
 * @param ErrorStatusITs 错误状�?
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
    // CAN总线离线, 重新启动CAN
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_Start(hfdcan);
  }
}
