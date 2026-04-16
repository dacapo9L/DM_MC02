#include "bmi088_gyro.h"

#include <math.h>
#include <string.h>

#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU
#define BMI088_GYRO_INIT_RETRY 20U
#define BMI088_GYRO_RW_RETRY 20U
#define BMI088_DEG_TO_RAD (3.14159265358979323846f / 180.0f)

static bool BMI088_Gyro_Read_Blocking(const Struct_BMI088_Gyro *gyro,
                                      uint8_t reg, uint8_t *data,
                                      uint16_t len) {
  uint8_t tx[32];
  uint8_t rx[32];
  uint16_t total;
  HAL_StatusTypeDef status;

  if (gyro == NULL || gyro->SPI_Manage_Object == NULL ||
      gyro->SPI_Manage_Object->SPI_Handler == NULL || data == NULL ||
      len == 0U) {
    return false;
  }

  total = (uint16_t)(1U + len);
  if (total > sizeof(tx)) {
    return false;
  }

  memset(tx, 0x55, total);
  memset(rx, 0x00, total);
  tx[0] = (uint8_t)(reg | BMI088_GYRO_READ_MASK);

  HAL_GPIO_WritePin(gyro->CS_GPIO_Port, gyro->CS_Pin, gyro->Activate_Pin_State);
  status = HAL_SPI_TransmitReceive(gyro->SPI_Manage_Object->SPI_Handler, tx, rx,
                                   total, 10U);
  HAL_GPIO_WritePin(gyro->CS_GPIO_Port, gyro->CS_Pin,
                    (gyro->Activate_Pin_State == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                               : GPIO_PIN_SET);

  if (status != HAL_OK) {
    return false;
  }

  memcpy(data, &rx[1U], len);
  return true;
}

static bool BMI088_Gyro_Write_Blocking(const Struct_BMI088_Gyro *gyro,
                                       uint8_t reg, uint8_t val) {
  uint8_t tx[2];
  HAL_StatusTypeDef status;

  if (gyro == NULL || gyro->SPI_Manage_Object == NULL ||
      gyro->SPI_Manage_Object->SPI_Handler == NULL) {
    return false;
  }

  tx[0] = reg;
  tx[1] = val;

  HAL_GPIO_WritePin(gyro->CS_GPIO_Port, gyro->CS_Pin, gyro->Activate_Pin_State);
  status = HAL_SPI_Transmit(gyro->SPI_Manage_Object->SPI_Handler, tx, 2U, 10U);
  HAL_GPIO_WritePin(gyro->CS_GPIO_Port, gyro->CS_Pin,
                    (gyro->Activate_Pin_State == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                               : GPIO_PIN_SET);

  return (status == HAL_OK);
}

static void BMI088_Gyro_Parse_Vector(Struct_BMI088_Gyro *gyro,
                                     const uint8_t *payload) {
  float scale;

  gyro->raw_gyro[0] = (int16_t)(((uint16_t)payload[1] << 8U) | payload[0]);
  gyro->raw_gyro[1] = (int16_t)(((uint16_t)payload[3] << 8U) | payload[2]);
  gyro->raw_gyro[2] = (int16_t)(((uint16_t)payload[5] << 8U) | payload[4]);

  scale = (float)(1U << (4U - (uint8_t)gyro->Range)) * 125.0f *
          BMI088_DEG_TO_RAD / 32768.0f;

  gyro->gyro_rads[0] = (float)gyro->raw_gyro[0] * scale;
  gyro->gyro_rads[1] = (float)gyro->raw_gyro[1] * scale;
  gyro->gyro_rads[2] = (float)gyro->raw_gyro[2] * scale;

  gyro->valid_flag = isfinite(gyro->gyro_rads[0]) &&
                     isfinite(gyro->gyro_rads[1]) &&
                     isfinite(gyro->gyro_rads[2]);
}

void BMI088_Gyro_Init(Struct_BMI088_Gyro *gyro,
                      Struct_SPI_Manage_Object *spi_manage_object) {
  if (gyro == NULL) {
    return;
  }

  memset(gyro, 0, sizeof(*gyro));

  gyro->SPI_Manage_Object = spi_manage_object;
  gyro->CS_GPIO_Port = BMI088_GYRO_CS_GPIO_Port;
  gyro->CS_Pin = BMI088_GYRO_CS_Pin;
  gyro->Activate_Pin_State = GPIO_PIN_RESET;
  gyro->Range = BMI088_GYRO_RANGE_2000DPS;
  gyro->valid_flag = false;
}

bool BMI088_Gyro_Configure(Struct_BMI088_Gyro *gyro) {
  uint8_t chip_id = 0U;
  uint8_t reg_val = 0U;
  uint32_t retry;
  uint8_t i;
  const Struct_BMI088_Gyro_Reg_Config init_cfg[] = {
      {BMI088_GYRO_REG_RANGE, (uint8_t)BMI088_GYRO_RANGE_2000DPS},
      {BMI088_GYRO_REG_BANDWIDTH, (uint8_t)(0x01U | 0x80U)},
      {BMI088_GYRO_REG_INT_CTRL, (uint8_t)(0x01U << 7U)},
      {BMI088_GYRO_REG_INT3_INT4_IO_CONF, 0x0CU},
      {BMI088_GYRO_REG_INT3_INT4_IO_MAP, 0x01U},
  };

  if (gyro == NULL) {
    return false;
  }

  for (retry = 0U; retry < BMI088_GYRO_INIT_RETRY; retry++) {
    if (BMI088_Gyro_Read_Blocking(gyro, BMI088_GYRO_REG_CHIP_ID, &chip_id,
                                  1U) &&
        chip_id == BMI088_GYRO_CHIP_ID_VALUE) {
      break;
    }
    HAL_Delay(2U);
  }
  if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
    return false;
  }

  if (!BMI088_Gyro_Write_Blocking(gyro, BMI088_GYRO_REG_SOFTRESET, 0xB6U)) {
    return false;
  }
  HAL_Delay(100U);

  chip_id = 0U;
  for (retry = 0U; retry < BMI088_GYRO_INIT_RETRY; retry++) {
    if (BMI088_Gyro_Read_Blocking(gyro, BMI088_GYRO_REG_CHIP_ID, &chip_id,
                                  1U) &&
        chip_id == BMI088_GYRO_CHIP_ID_VALUE) {
      break;
    }
    HAL_Delay(2U);
  }
  if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
    return false;
  }

  for (i = 0U; i < (uint8_t)(sizeof(init_cfg) / sizeof(init_cfg[0])); i++) {
    bool reg_ok = false;

    for (retry = 0U; retry < BMI088_GYRO_RW_RETRY; retry++) {
      if (!BMI088_Gyro_Write_Blocking(gyro, init_cfg[i].reg, init_cfg[i].val)) {
        HAL_Delay(2U);
        continue;
      }
      HAL_Delay(2U);
      if (BMI088_Gyro_Read_Blocking(gyro, init_cfg[i].reg, &reg_val, 1U) &&
          reg_val == init_cfg[i].val) {
        reg_ok = true;
        break;
      }
      HAL_Delay(2U);
    }

    if (!reg_ok) {
      return false;
    }
  }

  return true;
}

uint8_t BMI088_Gyro_Request_Gyro(Struct_BMI088_Gyro *gyro) {
  if (gyro == NULL || gyro->SPI_Manage_Object == NULL ||
      gyro->SPI_Manage_Object->SPI_Handler == NULL) {
    return HAL_ERROR;
  }

  gyro->SPI_Manage_Object->Tx_Buffer[0] =
      (uint8_t)(BMI088_GYRO_REG_RATE_X_LSB | BMI088_GYRO_READ_MASK);

  return SPI_Transmit_Receive_Data(gyro->SPI_Manage_Object->SPI_Handler,
                                   gyro->CS_GPIO_Port, gyro->CS_Pin,
                                   gyro->Activate_Pin_State, 1U, 6U);
}

void BMI088_Gyro_SPI_RxCpltCallback(Struct_BMI088_Gyro *gyro,
                                    const uint8_t *tx_buffer,
                                    const uint8_t *rx_buffer, uint16_t tx_len,
                                    uint16_t rx_len) {
  const uint8_t *payload;
  uint8_t reg;

  if (gyro == NULL || tx_buffer == NULL || rx_buffer == NULL || tx_len == 0U ||
      rx_len == 0U) {
    return;
  }

  reg = (uint8_t)(tx_buffer[0] & (uint8_t)(~BMI088_GYRO_READ_MASK));
  payload = &rx_buffer[1U];

  if (reg == BMI088_GYRO_REG_RATE_X_LSB && rx_len >= 6U) {
    BMI088_Gyro_Parse_Vector(gyro, payload);
  }
}
