/*
 * @file bno085_native_hal_stub.h
 * @brief Host-side stand-ins for the STM32 HAL types named by this library's
 * public headers.
 *
 * Selected by defining BNO085_BUILD_WITH_NATIVE_HAL_STUB. Intended for host
 * builds that link MockBNO085_SPI_Library instead of the real library: every
 * library function is mocked, so only the types appearing in the public
 * signatures have to exist. No HAL function is declared here on purpose, so a
 * test that accidentally reaches the HAL fails to link instead of silently
 * calling into a stub.
 *
 * Lives beside stm32_hal.h, which includes it, so that every translation unit
 * able to find the public headers can also find this one. In mock/ it would
 * only have been reachable from code that already depends on the mock library.
 */

#ifndef INCLUDE_BNO085_NATIVE_HAL_STUB_H_
#define INCLUDE_BNO085_NATIVE_HAL_STUB_H_

#include <stdint.h>

// The library only ever passes these around as pointers. They carry a member
// because CMock compares struct contents by value when a test dereferences
// them, and empty structs are not valid C.
typedef struct {
  int unused;
} GPIO_TypeDef;

typedef struct {
  int unused;
} SPI_TypeDef;

typedef struct {
  SPI_TypeDef *Instance;
} SPI_HandleTypeDef;

#endif  // INCLUDE_BNO085_NATIVE_HAL_STUB_H_
