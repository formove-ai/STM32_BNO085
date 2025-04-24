# BNO085/086 SPI library for STM

## Configuration

Two configuration structs: One for global SPI assignments, one for sensors

- Add to sensor struct
  - RSTN_PORT, RSTN_PIN

## Shims

- Implement `delay_U`

## Prerequisites

The user needs to define a compelte timer for SPI, max 3Mhz

- Timers for relevant GPIO lines must be enabled (e.g. A,B,C,D)
  __HAL_RCC_TIM2_CLK_ENABLE();
- Four PINs initialized, example from HAL_Init_GPIO_static

## Example for configuring PINs

```c
/**
 * @brief Init static GPIOs for SPI communication
 * @note HAL_GPIO_Init doesn't have a return value like an error flag
 */
static void HAL_Init_GPIO_static(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure PS0*/
  GPIO_InitStruct.Pin = PS0_PIN_S1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PS0_PORT_S1, PS0_PIN_S1, GPIO_PIN_SET);

  /* Configure PS1*/
  GPIO_InitStruct.Pin = PS1_PIN_S1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(PS1_PORT_S1, PS1_PIN_S1, GPIO_PIN_SET);

  /* Configure CLKSEL*/
  GPIO_InitStruct.Pin = CLKSEL_PIN_S1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(CLKSEL_PORT_S1, CLKSEL_PIN_S1, GPIO_PIN_RESET);

  /* Configure BOOTN*/
  GPIO_InitStruct.Pin = BOOTN_PIN_S1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTN_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BOOTN_PORT_S1, BOOTN_PIN_S1, GPIO_PIN_SET);
}
```
