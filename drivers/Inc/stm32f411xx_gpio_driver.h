/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jun 26, 2025
 *      Author: omega
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_
#include "stm32f411xx.h"

typedef struct {
  uint8_t GPIO_PinNumber;
  uint8_t GPIO_PinMode; //  @GPIO_PIN_MODE
  uint8_t GPIO_PinSpeed;
  uint8_t GPIO_PinPuPdControl;
  uint8_t GPIO_PinOPType;
  uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
  GPIO_RegDef_t *pGPIOx;
  GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

// @GPIO_PIN_MODE
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

// GPIO pin output types
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// @GPIO_PIN_SPEED
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

// GPIO pint pull up and pull down configuration marcro
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

// API support by this driver
// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// Init and deinit GPIO
void GPIO_Init(GPIO_Handle_t *pGPIOx);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Read and write data
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
                          uint8_t Value);
void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t Value);

// IRQ configuration and  handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIOIRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
