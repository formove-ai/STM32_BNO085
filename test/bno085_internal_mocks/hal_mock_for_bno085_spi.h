#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HAL_MOCK_FOR_BNO085_SPI_H
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HAL_MOCK_FOR_BNO085_SPI_H

#include <stdint.h>

// Everything to mock goes here. That is all HAL function that are used by
// modules under test.

typedef enum {
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/**
 * @brief  FDCAN Rx header structure definition
 */
typedef struct {
  uint32_t Identifier;

  uint32_t IdType;
  uint32_t RxFrameType;

  uint32_t DataLength;

  uint32_t ErrorStateIndicator;

  uint32_t BitRateSwitch;

  uint32_t FDFormat;

  uint32_t RxTimestamp;

  uint32_t FilterIndex;

  uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

/**
 * @brief  FDCAN Tx header structure definition
 */
typedef struct {
  uint32_t Identifier;

  uint32_t IdType;

  uint32_t TxFrameType;

  uint32_t DataLength;

  uint32_t ErrorStateIndicator;

  uint32_t BitRateSwitch;

  uint32_t FDFormat;
  uint32_t TxEventFifoControl;
  uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
  int foo;  // Avoid the struct being empty, as cmock comparison then fails.
} GPIO_TypeDef;

// A fake for the FDCAN handle. What it contains is currently not important for
// our tests as the libraries only use it as a reference.
typedef struct {
  int foo;  // Avoid the struct being empty, as cmock comparison then fails.
} FDCAN_HandleTypeDef;

typedef struct {
  uint32_t Mode;

  uint32_t Direction;

  uint32_t DataSize;

  uint32_t CLKPolarity;

  uint32_t CLKPhase;

  uint32_t NSS;

  uint32_t BaudRatePrescaler;

  uint32_t FirstBit;
  uint32_t TIMode;

  uint32_t CRCCalculation;

  uint32_t CRCPolynomial;

  uint32_t CRCLength;

  uint32_t NSSPMode;
} SPI_InitTypeDef;

typedef struct {
  uint32_t CR1;
  uint32_t CR2;
  uint32_t SR;
  uint32_t DR;
  uint32_t CRCPR;
  uint32_t RXCRCR;
  uint32_t TXCRCR;
  uint32_t I2SCFGR;
  uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
  int foo;
} DMA_HandleTypeDef;

typedef enum { HAL_UNLOCKED = 0x00U, HAL_LOCKED = 0x01U } HAL_LockTypeDef;

typedef enum {
  HAL_SPI_STATE_RESET = 0x00U,
  HAL_SPI_STATE_READY = 0x01U,
  HAL_SPI_STATE_BUSY = 0x02U,
  HAL_SPI_STATE_BUSY_TX = 0x03U,
  HAL_SPI_STATE_BUSY_RX = 0x04U,
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,
  HAL_SPI_STATE_ERROR = 0x06U,
  HAL_SPI_STATE_ABORT = 0x07U
} HAL_SPI_StateTypeDef;

typedef struct __SPI_HandleTypeDef {
  SPI_TypeDef *Instance;

  SPI_InitTypeDef Init;

  uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  uint16_t RxXferCount;

  uint32_t CRCSize;

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  HAL_SPI_StateTypeDef State;

  uint32_t ErrorCode;
} SPI_HandleTypeDef;
// A fake for the ADC handle. What it contains is currently not important for
// our tests as the libraries only use it as a reference.
typedef struct {
  int foo;  // Avoid the struct being empty, as cmock comparison then fails.
} ADC_HandleTypeDef;

typedef struct {
  uint32_t CR1;
  uint32_t CR2;
  uint32_t SMCR;
  uint32_t DIER;
  uint32_t SR;
  uint32_t EGR;
  uint32_t CCMR1;
  uint32_t CCMR2;
  uint32_t CCER;
  uint32_t CNT;
  uint32_t PSC;
  uint32_t ARR;
  uint32_t RCR;
  uint32_t CCR1;
  uint32_t CCR2;
  uint32_t CCR3;
  uint32_t CCR4;
  uint32_t BDTR;
  uint32_t DCR;
  uint32_t DMAR;
  uint32_t OR1;
  uint32_t CCMR3;
  uint32_t CCR5;
  uint32_t CCR6;
  uint32_t AF1;
  uint32_t AF2;
  uint32_t TISEL;
} TIM_TypeDef;

typedef struct {
  uint32_t Prescaler;

  uint32_t CounterMode;
  uint32_t Period;

  uint32_t ClockDivision;
  uint32_t RepetitionCounter;

  uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
  TIM_TypeDef *Instance;
  TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET } GPIO_PinState;

typedef struct {
  uint32_t Pin;

  uint32_t Mode;

  uint32_t Pull;

  uint32_t Speed;

  uint32_t Alternate;
} GPIO_InitTypeDef;

// We cannot set a dangling pointer here as the actual contents of the
// structures will be checked by Unity in some cases.
extern GPIO_TypeDef PORT_CONTENTS;
#define MAX_INOKB_GPIO_PORT &PORT_CONTENTS

// Set some needed defines with dummy values. These are absolutely arbitrary,
// ordered by the point in time where we needed them to make tests work.
// Here, it is assumed that it is not important what the actual value is and
// that the tests reuse the fake values. We can change this in the future to
// have the test closer to production. Some code might make assumptions about
// the values here.
#define MAX_INOKB_GPIO_PIN 1
#define LED_ROT2_GPIO_PORT ((GPIO_TypeDef *)2)
#define MAX_STAT_PIN 3
#define LED_ROT1_PIN 4
#define LED_ROT1_GPIO_PORT ((GPIO_TypeDef *)5)
#define TIM_CHANNEL_4 6
#define SET 7
#define LED_ROT2_PIN 8
#define MAX_INOKB_PIN 9
#define MAX_STAT_GPIO_PORT ((GPIO_TypeDef *)10)
#define RESET 11
#define GPIO_PIN_15 12
#define FDCAN_STANDARD_ID 13
#define FDCAN_DLC_BYTES_12 14
#define FDCAN_ESI_ACTIVE 15
#define FDCAN_BRS_ON 16
#define FDCAN_FD_CAN 17
#define FDCAN_NO_TX_EVENTS 18
#define FDCAN_DATA_FRAME 19
#define FDCAN_RX_FIFO0 20
#define GPIOB ((GPIO_TypeDef *)21)
#define TIM2 ((TIM_TypeDef *)22)
#define TIM_CLOCKDIVISION_DIV1 23
#define TIM_COUNTERMODE_UP 24
#define GPIO_MODE_INPUT 25
#define GPIO_PULLUP 26
#define GPIO_MODE_OUTPUT_PP 27
#define GPIO_SPEED_FREQ_VERY_HIGH 28
#define GPIO_PIN_9 29
#define GPIO_SPEED_FREQ_LOW 30
#define GPIO_PIN_7 31
#define GPIO_NOPULL 32
#define GPIOC ((GPIO_TypeDef *)33)
#define GPIO_PIN_6 34
#define GPIO_PIN_5 35
#define GPIO_PIN_4 36
#define GPIO_PIN_3 37
#define GPIO_PIN_2 38
#define GPIO_PIN_1 39
#define GPIOA ((GPIO_TypeDef *)40)
#define GPIO_MODE_AF_PP 41
#define GPIO_AF5_SPI1 42
#define GPIO_AF5_SPI2 43
#define SPI1 ((SPI_TypeDef *)44)
#define SPI2 ((SPI_TypeDef *)45)
#define SPI_MODE_MASTER 46
#define SPI_DIRECTION_2LINES 47
#define SPI_DATASIZE_8BIT 48
#define SPI_POLARITY_HIGH 49
#define SPI_PHASE_2EDGE 50
#define SPI_NSS_SOFT 51
#define SPI_FIRSTBIT_MSB 52
#define SPI_TIMODE_DISABLE 53
#define SPI_CRCCALCULATION_DISABLE 54
#define HAL_MAX_DELAY 55
#define FDCAN_DLC_BYTES_20 56
#define GPIO_PIN_11 57
#define GPIO_PIN_13 58
#define GPIO_PIN_14 59
#define GPIO_PIN_12 60
#define GPIO_PIN_0 61
#define GPIOD ((GPIO_TypeDef *)62)
#define SPI_BAUDRATEPRESCALER_16 63
#define TIM_AUTORELOAD_PRELOAD_DISABLE 64

// CAN-related functions
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t RxFifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t RxLocation,
                                         FDCAN_RxHeaderTypeDef *pRxHeader,
                                         uint8_t *pRxData);

// SPI-related functions
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                       uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                              uint8_t *pTxData,
                                              uint8_t *pRxData, uint16_t Size);

// GPIO-related functions
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                       GPIO_PinState PinState);
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);

// Timer and tick related functions
void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef *htim, uint32_t Channel,
                           uint32_t Compare);
uint32_t HAL_GetTick(void);
int __HAL_RCC_TIM2_CLK_ENABLE();
uint32_t HAL_RCC_GetHCLKFreq(void);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
void __HAL_RCC_GPIOA_CLK_ENABLE();
void __HAL_RCC_GPIOB_CLK_ENABLE();
void __HAL_RCC_GPIOC_CLK_ENABLE();
void __HAL_RCC_GPIOD_CLK_ENABLE();
void __HAL_RCC_SPI1_CLK_ENABLE();
void __HAL_RCC_SPI2_CLK_ENABLE();
int __HAL_TIM_GET_COUNTER(TIM_HandleTypeDef *htim);

// ADC-related functions
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc,
                                            uint32_t Timeout);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);

void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


#endif // SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_HAL_MOCK_FOR_BNO085_SPI_H
