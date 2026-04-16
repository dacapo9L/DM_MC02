#include "bmi088_accel.h"

#define BMI088_GRAVITY_ACCELERATION 9.7883f
#define BMI088_ACCEL_CHIP_ID_VALUE 0x1EU
#define BMI088_ACCEL_INIT_RETRY 20U
#define BMI088_ACCEL_RW_RETRY 20U

static bool BMI088_Accel_Read_Blocking(const Struct_BMI088_Accel *accel,
                                       uint8_t reg, uint8_t *data,
                                       uint16_t len) {
  uint8_t tx[32];
  uint8_t rx[32];
  uint16_t total;
  HAL_StatusTypeDef status;

  if (accel == NULL || accel->SPI_Manage_Object == NULL ||
      accel->SPI_Manage_Object->SPI_Handler == NULL || data == NULL ||
      len == 0U) {
    return false;
  }

  total = (uint16_t)(1U + BMI088_ACCEL_SPI_RX_RESERVED + len);
  if (total > sizeof(tx)) {
    return false;
  }

  memset(tx, 0x55, total);
  memset(rx, 0x00, total);
  tx[0] = (uint8_t)(reg | BMI088_ACCEL_READ_MASK);

  HAL_GPIO_WritePin(accel->CS_GPIO_Port, accel->CS_Pin,
                    accel->Activate_Pin_State);
  status = HAL_SPI_TransmitReceive(accel->SPI_Manage_Object->SPI_Handler, tx,
                                   rx, total, 10U);
  HAL_GPIO_WritePin(accel->CS_GPIO_Port, accel->CS_Pin,
                    (accel->Activate_Pin_State == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                                : GPIO_PIN_SET);

  if (status != HAL_OK) {
    return false;
  }

  memcpy(data, &rx[1U + BMI088_ACCEL_SPI_RX_RESERVED], len);
  return true;
}

static bool BMI088_Accel_Write_Blocking(const Struct_BMI088_Accel *accel,
                                        uint8_t reg, uint8_t val) {
  uint8_t tx[2];
  HAL_StatusTypeDef status;

  if (accel == NULL || accel->SPI_Manage_Object == NULL ||
      accel->SPI_Manage_Object->SPI_Handler == NULL) {
    return false;
  }

  tx[0] = reg;
  tx[1] = val;

  HAL_GPIO_WritePin(accel->CS_GPIO_Port, accel->CS_Pin,
                    accel->Activate_Pin_State);
  status = HAL_SPI_Transmit(accel->SPI_Manage_Object->SPI_Handler, tx, 2U, 10U);
  HAL_GPIO_WritePin(accel->CS_GPIO_Port, accel->CS_Pin,
                    (accel->Activate_Pin_State == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                                : GPIO_PIN_SET);

  return (status == HAL_OK);
}

static void BMI088_Accel_Parse_Vector(Struct_BMI088_Accel *accel,
                                      const uint8_t *payload) {
  float scale;

  accel->raw_accel[0] = (int16_t)(((uint16_t)payload[1] << 8U) | payload[0]);
  accel->raw_accel[1] = (int16_t)(((uint16_t)payload[3] << 8U) | payload[2]);
  accel->raw_accel[2] = (int16_t)(((uint16_t)payload[5] << 8U) | payload[4]);

  scale = (float)((1U << ((uint8_t)accel->Range + 1U)) * 1.5f) *
          BMI088_GRAVITY_ACCELERATION / 32768.0f;

  accel->accel_mps2[0] = (float)accel->raw_accel[0] * scale;
  accel->accel_mps2[1] = (float)accel->raw_accel[1] * scale;
  accel->accel_mps2[2] = (float)accel->raw_accel[2] * scale;

  accel->valid_flag = isfinite(accel->accel_mps2[0]) &&
                      isfinite(accel->accel_mps2[1]) &&
                      isfinite(accel->accel_mps2[2]);
}

static void BMI088_Accel_Parse_Temperature(Struct_BMI088_Accel *accel,
                                           const uint8_t *payload) {
  int16_t tmp;

  tmp = (int16_t)(((uint16_t)payload[0] << 3U) | ((uint16_t)payload[1] >> 5U));
  if ((tmp & 0x0400) != 0) {
    tmp = (int16_t)(tmp - (1 << 11));
  }

  accel->temperature_c = 23.0f + (float)tmp * 0.125f;
}

void BMI088_Accel_Init(Struct_BMI088_Accel *accel,
                       Struct_SPI_Manage_Object *spi_manage_object) {
  if (accel == NULL) {
    return;
  }

  memset(accel, 0, sizeof(*accel));

  accel->SPI_Manage_Object = spi_manage_object;
  accel->CS_GPIO_Port = BMI088_ACCEL_CS_GPIO_Port;
  accel->CS_Pin = BMI088_ACCEL_CS_Pin;
  accel->Activate_Pin_State = GPIO_PIN_RESET;
  accel->Range = BMI088_ACCEL_RANGE_6G;
  accel->valid_flag = false;
}

bool BMI088_Accel_Configure(Struct_BMI088_Accel *accel) {
  uint8_t chip_id = 0U;
  uint8_t reg_val = 0U;
  uint32_t retry;
  uint8_t i;
  uint8_t init_range;
  const Struct_BMI088_Accel_Reg_Config init_cfg_base[] = {
      {BMI088_ACCEL_REG_ACC_PWR_CTRL, 0x04U},
      {BMI088_ACCEL_REG_ACC_PWR_CONF, 0x00U},
      {BMI088_ACCEL_REG_ACC_CONF, (uint8_t)((0x0AU << 4U) | 0x0CU)},
      {BMI088_ACCEL_REG_ACC_RANGE, 0x00U},
      {BMI088_ACCEL_REG_INT1_IO_CTRL, (uint8_t)(0x01U << 3U)},
      {BMI088_ACCEL_REG_INT_MAP_DATA, (uint8_t)(0x01U << 2U)},
  };
  Struct_BMI088_Accel_Reg_Config
      init_cfg[sizeof(init_cfg_base) / sizeof(init_cfg_base[0])];

  if (accel == NULL) {
    return false;
  }

  memcpy(init_cfg, init_cfg_base, sizeof(init_cfg_base));
  init_range = (uint8_t)accel->Range;
  init_cfg[3].val = init_range;

  for (retry = 0U; retry < BMI088_ACCEL_INIT_RETRY; retry++) {
    if (BMI088_Accel_Read_Blocking(accel, BMI088_ACCEL_REG_CHIP_ID, &chip_id,
                                   1U) &&
        chip_id == BMI088_ACCEL_CHIP_ID_VALUE) {
      break;
    }
    HAL_Delay(2U);
  }
  if (chip_id != BMI088_ACCEL_CHIP_ID_VALUE) {
    return false;
  }

  if (!BMI088_Accel_Write_Blocking(accel, BMI088_ACCEL_REG_ACC_PWR_CTRL,
                                   0xB6U)) {
    return false;
  }
  HAL_Delay(100U);

  chip_id = 0U;
  for (retry = 0U; retry < BMI088_ACCEL_INIT_RETRY; retry++) {
    if (BMI088_Accel_Read_Blocking(accel, BMI088_ACCEL_REG_CHIP_ID, &chip_id,
                                   1U) &&
        chip_id == BMI088_ACCEL_CHIP_ID_VALUE) {
      break;
    }
    HAL_Delay(2U);
  }
  if (chip_id != BMI088_ACCEL_CHIP_ID_VALUE) {
    return false;
  }

  for (i = 0U; i < (uint8_t)(sizeof(init_cfg) / sizeof(init_cfg[0])); i++) {
    bool reg_ok = false;

    for (retry = 0U; retry < BMI088_ACCEL_RW_RETRY; retry++) {
      if (!BMI088_Accel_Write_Blocking(accel, init_cfg[i].reg,
                                       init_cfg[i].val)) {
        HAL_Delay(2U);
        continue;
      }
      HAL_Delay(2U);
      if (BMI088_Accel_Read_Blocking(accel, init_cfg[i].reg, &reg_val, 1U) &&
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

uint8_t BMI088_Accel_Request_Accel(Struct_BMI088_Accel *accel) {
  if (accel == NULL || accel->SPI_Manage_Object == NULL ||
      accel->SPI_Manage_Object->SPI_Handler == NULL) {
    return HAL_ERROR;
  }

  accel->SPI_Manage_Object->Tx_Buffer[0] =
      (uint8_t)(BMI088_ACCEL_REG_ACC_X_LSB | BMI088_ACCEL_READ_MASK);

  return SPI_Transmit_Receive_Data(
      accel->SPI_Manage_Object->SPI_Handler, accel->CS_GPIO_Port, accel->CS_Pin,
      accel->Activate_Pin_State, (uint16_t)(1U + BMI088_ACCEL_SPI_RX_RESERVED),
      6U);
}

uint8_t BMI088_Accel_Request_Temperature(Struct_BMI088_Accel *accel) {
  if (accel == NULL || accel->SPI_Manage_Object == NULL ||
      accel->SPI_Manage_Object->SPI_Handler == NULL) {
    return HAL_ERROR;
  }

  accel->SPI_Manage_Object->Tx_Buffer[0] =
      (uint8_t)(BMI088_ACCEL_REG_TEMP_MSB | BMI088_ACCEL_READ_MASK);

  return SPI_Transmit_Receive_Data(
      accel->SPI_Manage_Object->SPI_Handler, accel->CS_GPIO_Port, accel->CS_Pin,
      accel->Activate_Pin_State, (uint16_t)(1U + BMI088_ACCEL_SPI_RX_RESERVED),
      2U);
}

void BMI088_Accel_SPI_RxCpltCallback(Struct_BMI088_Accel *accel,
                                     const uint8_t *tx_buffer,
                                     const uint8_t *rx_buffer, uint16_t tx_len,
                                     uint16_t rx_len) {
  const uint8_t *payload;
  uint8_t reg;

  if (accel == NULL || tx_buffer == NULL || rx_buffer == NULL || tx_len == 0U ||
      rx_len == 0U) {
    return;
  }

  reg = (uint8_t)(tx_buffer[0] & (uint8_t)(~BMI088_ACCEL_READ_MASK));
  payload = &rx_buffer[1U + BMI088_ACCEL_SPI_RX_RESERVED];

  if (reg == BMI088_ACCEL_REG_ACC_X_LSB && rx_len >= 6U) {
    BMI088_Accel_Parse_Vector(accel, payload);
  } else if (reg == BMI088_ACCEL_REG_TEMP_MSB && rx_len >= 2U) {
    BMI088_Accel_Parse_Temperature(accel, payload);
  }
}
