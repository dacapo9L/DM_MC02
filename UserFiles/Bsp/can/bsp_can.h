#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <string.h>


/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct Struct_CAN_Manage_Object {
  FDCAN_HandleTypeDef *CAN_Handler;
  CAN_Callback Callback_Function;

  // 与接收相关的数据
  FDCAN_RxHeaderTypeDef Rx_Header;
  uint8_t Rx_Buffer[64];
  // 接收时间戳
  uint64_t Rx_Timestamp;
} Struct_CAN_Manage_Object;

// extern bool init_finished;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

// DJI
extern uint8_t CAN1_0x1fe_Tx_Data[];
extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2fe_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0x3fe_Tx_Data[];
extern uint8_t CAN1_0x4fe_Tx_Data[];

// ZDT
extern uint8_t CAN1_0x100_Tx_Data[];
extern uint8_t CAN1_0x101_Tx_Data[];
extern uint8_t CAN1_0x300_Tx_Data[];
extern uint8_t CAN1_0x301_Tx_Data[];
extern uint8_t CAN1_0x400_Tx_Data[];
extern uint8_t CAN1_0x401_Tx_Data[];
extern uint8_t CAN1_0x500_Tx_Data[];
extern uint8_t CAN1_0x501_Tx_Data[];

// DJI
extern uint8_t CAN2_0x1fe_Tx_Data[];
extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2fe_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0x3fe_Tx_Data[];
extern uint8_t CAN2_0x4fe_Tx_Data[];

// ZDT
extern uint8_t CAN2_0x100_Tx_Data[];
extern uint8_t CAN2_0x101_Tx_Data[];
extern uint8_t CAN2_0x300_Tx_Data[];
extern uint8_t CAN2_0x301_Tx_Data[];
extern uint8_t CAN2_0x400_Tx_Data[];
extern uint8_t CAN2_0x401_Tx_Data[];
extern uint8_t CAN2_0x500_Tx_Data[];
extern uint8_t CAN2_0x501_Tx_Data[];

// DJI
extern uint8_t CAN3_0x1fe_Tx_Data[];
extern uint8_t CAN3_0x1ff_Tx_Data[];
extern uint8_t CAN3_0x200_Tx_Data[];
extern uint8_t CAN3_0x2fe_Tx_Data[];
extern uint8_t CAN3_0x2ff_Tx_Data[];
extern uint8_t CAN3_0x3fe_Tx_Data[];
extern uint8_t CAN3_0x4fe_Tx_Data[];

// ZDT
extern uint8_t CAN3_0x100_Tx_Data[];
extern uint8_t CAN3_0x101_Tx_Data[];
extern uint8_t CAN3_0x300_Tx_Data[];
extern uint8_t CAN3_0x301_Tx_Data[];
extern uint8_t CAN3_0x400_Tx_Data[];
extern uint8_t CAN3_0x401_Tx_Data[];
extern uint8_t CAN3_0x500_Tx_Data[];
extern uint8_t CAN3_0x501_Tx_Data[];

extern uint8_t CAN_Supercap_Tx_Data[];

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, bool extid,
              CAN_Callback Callback_Function);

uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, bool exid,
                          uint8_t *Data, uint16_t Length);

void TIM_100us_CAN_PeriodElapsedCallback();

void TIM_1ms_CAN_PeriodElapsedCallback();

#endif
