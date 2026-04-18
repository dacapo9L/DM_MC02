#ifndef TASK_AND_CALLBACK_H
#define TASK_AND_CALLBACK_H

#include "bmi088.h"
#include "bmi088_heater.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
#include "buzzer.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "ins_AHRS.h"
#include "main.h"
#include "tim.h"
#include "ws2812.h"
#include "zdt_steppermotor.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void Task_Init(void);

#endif
