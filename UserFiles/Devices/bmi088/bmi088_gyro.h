#ifndef BMI088_GYRO_H
#define BMI088_GYRO_H

#include "bsp_spi.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define BMI088_GYRO_READ_MASK 0x80U

#define BMI088_GYRO_REG_CHIP_ID 0x00U
#define BMI088_GYRO_REG_RATE_X_LSB 0x02U
#define BMI088_GYRO_REG_RANGE 0x0FU
#define BMI088_GYRO_REG_BANDWIDTH 0x10U
#define BMI088_GYRO_REG_SOFTRESET 0x14U
#define BMI088_GYRO_REG_INT_CTRL 0x15U
#define BMI088_GYRO_REG_INT3_INT4_IO_CONF 0x16U
#define BMI088_GYRO_REG_INT3_INT4_IO_MAP 0x18U

typedef enum Enum_BMI088_Gyro_Range {
  BMI088_GYRO_RANGE_2000DPS = 0x00U,
  BMI088_GYRO_RANGE_1000DPS = 0x01U,
  BMI088_GYRO_RANGE_500DPS = 0x02U,
  BMI088_GYRO_RANGE_250DPS = 0x03U,
  BMI088_GYRO_RANGE_125DPS = 0x04U,
} Enum_BMI088_Gyro_Range;

typedef struct Struct_BMI088_Gyro {
  Struct_SPI_Manage_Object *SPI_Manage_Object;
  GPIO_TypeDef *CS_GPIO_Port;
  uint16_t CS_Pin;
  GPIO_PinState Activate_Pin_State;

  Enum_BMI088_Gyro_Range Range;

  bool valid_flag;
  int16_t raw_gyro[3];
  float gyro_rads[3];
} Struct_BMI088_Gyro;

typedef struct Struct_BMI088_Gyro_Reg_Config {
  uint8_t reg;
  uint8_t val;
} Struct_BMI088_Gyro_Reg_Config;

void BMI088_Gyro_Init(Struct_BMI088_Gyro *gyro,
                      Struct_SPI_Manage_Object *spi_manage_object);
bool BMI088_Gyro_Configure(Struct_BMI088_Gyro *gyro);

uint8_t BMI088_Gyro_Request_Gyro(Struct_BMI088_Gyro *gyro);
void BMI088_Gyro_SPI_RxCpltCallback(Struct_BMI088_Gyro *gyro,
                                    const uint8_t *tx_buffer,
                                    const uint8_t *rx_buffer, uint16_t tx_len,
                                    uint16_t rx_len);

#endif
