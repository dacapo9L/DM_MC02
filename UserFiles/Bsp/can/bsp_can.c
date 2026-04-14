#include "bsp_can.h"
#include "bsp_dwt.h"
#include "dji_motor.h"

Struct_CAN_Manage_Object CAN1_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN2_Manage_Object = {NULL};
Struct_CAN_Manage_Object CAN3_Manage_Object = {NULL};

// CAN通信发送缓冲区

// 电机共享区

// CAN1
uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

// CAN2
uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

// CAN3
uint8_t CAN3_0x1fe_Tx_Data[8];
uint8_t CAN3_0x1ff_Tx_Data[8];
uint8_t CAN3_0x200_Tx_Data[8];
uint8_t CAN3_0x2fe_Tx_Data[8];
uint8_t CAN3_0x2ff_Tx_Data[8];
uint8_t CAN3_0x3fe_Tx_Data[8];
uint8_t CAN3_0x4fe_Tx_Data[8];

/**
 * @brief 配置CAN的过滤器
 * 默认开了fifo0和fifo1的全通滤波器, 但由于fifo0和fifo1匹配规则一致
 * 因此fifo1理论上不会被触发, 即使fifo0满。如若出现接收满的情况,
 * 可配置掩码选择性接收, 合理分担总线带宽。此外,
 * Cortex-M7内核的滤波器配置在每个CAN实例都是独立的, 且滤波器编号也都是独立的
 * 如: F4系列芯片的CAN1和CAN2的滤波器编号分别是0-13和14-27,
 * 但H7系列芯片的FDCAN1和FDCAN2的滤波器编号都可以从0开始
 *
 * @param hfdcan CAN编号
 */
void can_filter_mask_config(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_FilterTypeDef can_filter_init_structure;

  // 配置fifo0全通滤波器
  can_filter_init_structure.IdType = FDCAN_STANDARD_ID;
  can_filter_init_structure.FilterIndex = 0;
  can_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
  can_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_init_structure.FilterID1 = 0x00000000;
  can_filter_init_structure.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(hfdcan, &can_filter_init_structure);

  // 全局滤波器: 直接拒绝不符合规则的标准数据帧、扩展数据帧、标准遥控帧
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
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function) {
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

  can_filter_mask_config(hfdcan);

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
uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID,
                          uint8_t *Data, uint16_t Length) {
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = ID;
  tx_header.IdType = FDCAN_STANDARD_ID;
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
  static int mod2 = 0;
  mod2++;
  if (mod2 == 2) {
    mod2 = 0;

    // 发送实例
    CAN_Transmit_Data(&hfdcan1, 0x1fe, CAN1_0x1fe_Tx_Data, 8);
  }
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hfdcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  // 判断程序初始化完成
  if (0) {
    // 也得接收, 防止FIFO满
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
 * @param ErrorStatusITs 错误状态
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
    // CAN总线离线, 重新启动CAN
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_Start(hfdcan);
  }
}
