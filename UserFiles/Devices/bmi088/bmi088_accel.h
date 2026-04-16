#ifndef BMI088_ACCEL_H
#define BMI088_ACCEL_H

#include "bsp_spi.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BMI088_ACCEL_READ_MASK 0x80U
#define BMI088_ACCEL_SPI_RX_RESERVED 1U

#define BMI088_ACCEL_REG_CHIP_ID 0x00U
#define BMI088_ACCEL_REG_ACC_X_LSB 0x12U
#define BMI088_ACCEL_REG_TEMP_MSB 0x22U
#define BMI088_ACCEL_REG_TEMP_LSB 0x23U
#define BMI088_ACCEL_REG_ACC_CONF 0x40U
#define BMI088_ACCEL_REG_ACC_RANGE 0x41U
#define BMI088_ACCEL_REG_INT1_IO_CTRL 0x53U
#define BMI088_ACCEL_REG_INT_MAP_DATA 0x58U
#define BMI088_ACCEL_REG_ACC_PWR_CONF 0x7CU
#define BMI088_ACCEL_REG_ACC_PWR_CTRL 0x7DU

typedef enum Enum_BMI088_Accel_Range {
  BMI088_ACCEL_RANGE_3G = 0x00U,
  BMI088_ACCEL_RANGE_6G = 0x01U,
  BMI088_ACCEL_RANGE_12G = 0x02U,
  BMI088_ACCEL_RANGE_24G = 0x03U,
} Enum_BMI088_Accel_Range;

typedef struct Struct_BMI088_Accel {
  Struct_SPI_Manage_Object *SPI_Manage_Object;
  GPIO_TypeDef *CS_GPIO_Port;
  uint16_t CS_Pin;
  GPIO_PinState Activate_Pin_State;

  Enum_BMI088_Accel_Range Range;

  bool valid_flag;
  int16_t raw_accel[3];
  float accel_mps2[3];
  float temperature_c;
} Struct_BMI088_Accel;

typedef struct Struct_BMI088_Accel_Reg_Config {
  uint8_t reg;
  uint8_t val;
} Struct_BMI088_Accel_Reg_Config;

void BMI088_Accel_Init(Struct_BMI088_Accel *accel,
                       Struct_SPI_Manage_Object *spi_manage_object);
bool BMI088_Accel_Configure(Struct_BMI088_Accel *accel);

uint8_t BMI088_Accel_Request_Accel(Struct_BMI088_Accel *accel);
uint8_t BMI088_Accel_Request_Temperature(Struct_BMI088_Accel *accel);
void BMI088_Accel_SPI_RxCpltCallback(Struct_BMI088_Accel *accel,
                                     const uint8_t *tx_buffer,
                                     const uint8_t *rx_buffer, uint16_t tx_len,
                                     uint16_t rx_len);

#endif
