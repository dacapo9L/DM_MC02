#ifndef BSP_DWT_H
#define BSP_DWT_H
#include "main.h"

extern void DWT_Init(uint32_t cpu_freq_mhz);
extern float DWT_Get_Delta_T(uint32_t *cnt_last);
extern float DWT_Get_Timestamp();
extern void DWT_Delay_us(float us);

#endif