/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 26, 2025
 *      Author: omega
 */

#include "../Inc/stm32f411xx_gpio_driver.h"

// API support by this driver
// Peripheral clock setup
// @fn              GPIO_PeriClockControl
// @param[in]       Base address of the gpio
// @param[in]       1 is enable, 0 is disable
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
  if (EnOrDi == ENABLE) {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_EN;
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_EN;
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_EN;
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_EN;
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_EN;
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_EN;
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_EN;
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_EN;
    } else if (pGPIOx == GPIOI) {
      GPIOI_PCLK_EN;
    } else if (pGPIOx == GPIOJ) {
      GPIOJ_PCLK_EN;
    } else if (pGPIOx == GPIOK) {
      GPIOK_PCLK_EN;
    }
  } else {
  }
};

// Init and deinit GPIO
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {
  uint32_t temp = 0;
  // 1. configure the mode of gpio pin
  if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
    temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode
            << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
  }
  // 2. configure the speed
  // 3. configure the pupd settings
  // 4. configure the optype
  // 5. configure the alt funtion
};
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
