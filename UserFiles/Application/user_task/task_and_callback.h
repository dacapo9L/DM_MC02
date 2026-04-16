#ifndef TASK_AND_CALLBACK_H
#define TASK_AND_CALLBACK_H

#include "bmi088.h"
#include "bmi088_heater.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "buzzer.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "ins_AHRS.h"
#include "main.h"
#include "tim.h"
#include "ws2812.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  uint16_t last_ecd;
} motor_measure_t;

// Global time base for INS and controllers (unit: 0.1 ms).
extern volatile uint32_t nowtime;

void CAN1_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Task_Init(void);

#endif
