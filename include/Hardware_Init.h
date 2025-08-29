/*
 * @file Hardware_Init.h
 * @brief HAL initialization functions for hardware (GPIO, Timer, SPI).
 *
 * See sensorsuit/experimental/imu_readout_library/README.md for more
 * information.
 */

#ifndef INCLUDE_HARDWARE_INIT_H_
#define INCLUDE_HARDWARE_INIT_H_

// --- Includes -------------------------------------------------------
// Includes of standard libraries
#include <stdint.h>
#include <stdio.h>

// Private includes
#include "BNO085_SPI_Error_Flags.h"
#include "Sensor_Struct.h"
#include "stm32_hal.h"
#include "stm32_hal_spi.h"

// --- Macros and data ------------------------------------------------

// SPI Peripheral handler, SPI1
extern SPI_HandleTypeDef hspi;

typedef struct bno085_library_spi_config_struct {
  GPIO_TypeDef *SPI_Port;
  uint16_t SPI_MISO_Pin;
  uint16_t SPI_MOSI_Pin;
  uint16_t SPI_SCK_Pin;
  SPI_TypeDef *SPI_Instance;
  uint32_t SPI_AF_mapping;
  uint32_t SPI_prescaler;
} bno085_library_spi_config_struct;

// --- Public functions -----------------------------------------------
// function prototypes
void init_GPIO_IMU(sensor_meta *sensor);

// Replace the aove with one function
uint8_t init_HardwareBNO085(
    SPI_HandleTypeDef *hspi,
    bno085_library_spi_config_struct bno085_library_spi_config);

#endif  // INCLUDE_HARDWARE_INIT_H_
