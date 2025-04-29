#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_STM32_HAL_SPI_H
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_STM32_HAL_SPI_H

// some random change

#if defined(STM32F0xx)
#include "stm32f0xx_hal_spi.h"
#elif defined(STM32F1xx)
#include "stm32f1xx_hal_spi.h"
#elif defined(STM32F2xx)
#include "stm32f2xx_hal_spi.h"
#elif defined(STM32F3xx)
#include "stm32f3xx_hal_spi.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal_spi.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal_spi.h"
#elif defined(STM32G0xx)
#include "stm32g0xx_hal_spi.h"
#elif defined(STM32G4xx)
#include "stm32g4xx_hal_spi.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal_spi.h"
#elif defined(STM32L0xx)
#include "stm32l0xx_hal_spi.h"
#elif defined(STM32L1xx)
#include "stm32l1xx_hal_spi.h"
#elif defined(STM32L4xx)
#include "stm32l4xx_hal_spi.h"
#elif defined(STM32L5xx)
#include "stm32l5xx_hal_spi.h"
#elif defined(STM32U5xx)
#include "stm32u5xx_hal_spi.h"
#elif defined(STM32WBxx)
#include "stm32wbxx_hal_spi.h"
#elif defined(STM32WLxx)
#include "stm32wlxx_hal_spi.h"
#elif defined(BNO085_BUILD_FOR_INTERNAL_UNIT_TESTS)
#include "hal_mock_for_bno085_spi.h"
#else
#error "Unsupported STM32 series. Please define the appropriate STM32 series macro."
#endif

#endif // SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_STM32_HAL_SPI_H
