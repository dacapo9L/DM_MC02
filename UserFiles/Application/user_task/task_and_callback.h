#ifndef TASK_AND_CALLBACK_H
#define TASK_AND_CALLBACK_H

#include "bsp_can.h"
#include "bsp_usb.h"
#include "dji_motor.h"
#include "buzzer.h"
#include "ws2812.h"
#include "fdcan.h"
#include "main.h"
#include "tim.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  uint16_t last_ecd;
} motor_measure_t;

void CAN1_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Task_Init(void);

#ifdef __cplusplus
}
#endif

#endif
