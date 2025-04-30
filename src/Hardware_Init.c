/*
 * @file Hardware_Init.c
 * @brief HAL initialization functions for hardware (GPIO, Timer, SPI).
 */

// --- Includes -------------------------------------------------------
#include "Hardware_Init.h"

// --- Macros and data ------------------------------------------------

bno085_library_spi_config_struct bno085_library_spi_config;
SPI_HandleTypeDef hspi;

// --- Private methods ------------------------------------------------

/**
 * @brief Init GPIOs for sensor
 * @note HAL_GPIO_Init doesn't have a return value like an error flag
 * @param ports_oins_config: Struct of ports and pins configuration of
 * corresponding sensor
 */
static void HAL_Init_GPIO_Sensor(ports_pins ports_pins_config) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : INTN */
  HAL_GPIO_WritePin(ports_pins_config.INTN_Port, ports_pins_config.INTN_Pin,
                    GPIO_PIN_SET);
  GPIO_InitStruct.Pin = ports_pins_config.INTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ports_pins_config.INTN_Port, &GPIO_InitStruct);

  /* Configure CSN */
  HAL_GPIO_WritePin(ports_pins_config.CSN_Port, ports_pins_config.CSN_Pin,
                    GPIO_PIN_SET);
  GPIO_InitStruct.Pin = ports_pins_config.CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ports_pins_config.CSN_Port, &GPIO_InitStruct);

  /* Configure RSTN*/
  HAL_GPIO_WritePin(ports_pins_config.RSTN_Port, ports_pins_config.RSTN_Pin,
                    GPIO_PIN_SET);
  GPIO_InitStruct.Pin = ports_pins_config.RSTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ports_pins_config.RSTN_Port, &GPIO_InitStruct);
}

/**
 * @brief Init SPI
 * @return status: 1 no error occurred, 0 an error occurred
 */
static uint8_t HAL_Init_SPI(
    bno085_library_spi_config_struct bno085_library_spi_config) {
  uint8_t status = HAL_OK;

  GPIO_InitTypeDef GPIO_InitStruct;

  // Check if the prescaler value is within the acceptable range (2, 4, 8, ...,
  // 256)
  if (bno085_library_spi_config.SPI_prescaler < 2) {
    bno085_library_spi_config.SPI_prescaler = 2;
  } else if (bno085_library_spi_config.SPI_prescaler > 256) {
    bno085_library_spi_config.SPI_prescaler = 256;
  }

  // SPI1 GPIO Configuration
  GPIO_InitStruct.Pin = bno085_library_spi_config.SPI_SCK_Pin |
                        bno085_library_spi_config.SPI_MISO_Pin |
                        bno085_library_spi_config.SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = bno085_library_spi_config.SPI_AF_mapping;
  HAL_GPIO_Init(bno085_library_spi_config.SPI_Port, &GPIO_InitStruct);

  // Init SPI 1 Peripheral
  hspi.Instance = bno085_library_spi_config.SPI_Instance;
  hspi.Init.Mode = SPI_MODE_MASTER;            // STM32 is master
  hspi.Init.Direction = SPI_DIRECTION_2LINES;  // Full duplex master
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;     // 8-bit segments (cf. [1], p. 19)
  hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;  // CPOL = 1 (cf. [1], p. 19)
  hspi.Init.CLKPhase = SPI_PHASE_2EDGE;       // CPHA = 1 (cf. [1], p. 19)
  hspi.Init.BaudRatePrescaler =
      bno085_library_spi_config
          .SPI_prescaler;  // Set prescaler to calculated value (cf. [1], p. 47)
  hspi.Init.NSS = SPI_NSS_SOFT;  // the Slave Select is handled manually
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;  // MSB first (cf. [1], p. 19)
  hspi.Init.TIMode =
      SPI_TIMODE_DISABLE;  // standard full-duplex mode, not 3 wire mode
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  // No checksum needed
  hspi.Init.CRCPolynomial = 10;  // placeholder, CRC disabled

  status = HAL_SPI_Init(&hspi);

  if (status == HAL_OK) {
    status = N_ERR;
  } else {
    status = D_ERR;
  }
  return status;
}

// --- Public functions -------------------------------------------------

/**
 * @brief Initializes GPIOs for IMUs
 * @param *sensor: Pointer to corresponding sensor meta data
 */
void init_GPIO_IMU(sensor_meta *sensor) {
  HAL_Init_GPIO_Sensor(sensor->ports_pins);
}

uint8_t init_HardwareBNO085(
    SPI_HandleTypeDef *hspi,
    bno085_library_spi_config_struct bno085_library_spi_config) {
  hspi = hspi;
  bno085_library_spi_config = bno085_library_spi_config;
  return HAL_Init_SPI(bno085_library_spi_config);
}
