// TODO(#7): Use cmock to mock HAL functions

#include "hal_mock_for_bno085_spi.h"

#include <string.h>

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                       GPIO_PinState PinState) {
  // Do nothing
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  return GPIO_PIN_RESET;
}

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
  // Do nothing
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                      uint16_t Size, uint32_t Timeout) {
  return HAL_OK;
}

// TransmitReceive
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData,
                                          uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout) {
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi) {
  return HAL_OK;
}

uint32_t HAL_GetTick(void) { return 0U; }

HAL_SPI_StateTypeDef HAL_SPI_GetState(const SPI_HandleTypeDef *hspi) {
  (void)hspi;
  return HAL_SPI_STATE_READY;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                              uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size) {
  if (hspi != NULL && pTxData != NULL && pRxData != NULL && Size > 0) {
    memcpy(pRxData, pTxData, (size_t)Size);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi) {
  (void)hspi;
  return HAL_OK;
}
