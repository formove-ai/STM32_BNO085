#include "hal_mock_for_bno085_spi.h"

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
