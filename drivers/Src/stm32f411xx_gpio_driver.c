/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 26, 2025
 *      Author: omega
 */

#include "../Inc/stm32f411xx_gpio_driver.h"
#include <stdint.h>

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
        temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIO_Handle->pGPIOx->MODER |= temp;
        // temp = 0;
    }
    else{
        
    }
    // 2. configure the speed
    temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
    // 3. configure the pupd settings
    temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIO_Handle->pGPIOx->PUPDR |= temp;
    // 4. configure the optype
    temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIO_Handle->pGPIOx->OTYPER |= temp;

    // 5. configure the alt funtion
    if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
        uint8_t temp1,temp2;
        temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
        pGPIO_Handle->pGPIOx->AFR[temp1] |= pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2);
    }
};
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
    if (pGPIOx == GPIOA) {
      GPIOA_REG_RESET;
    } else if (pGPIOx == GPIOB) {
      GPIOB_REG_RESET;
    } else if (pGPIOx == GPIOC) {
      GPIOC_REG_RESET;
    } else if (pGPIOx == GPIOD) {
      GPIOD_REG_RESET;
    } else if (pGPIOx == GPIOE) {
      GPIOE_REG_RESET;
    } else if (pGPIOx == GPIOF) {
      GPIOF_REG_RESET;
    } else if (pGPIOx == GPIOG) {
      GPIOG_REG_RESET;
    } else if (pGPIOx == GPIOH) {
      GPIOH_REG_RESET;
    } else if (pGPIOx == GPIOI) {
      GPIOI_REG_RESET;
    } else if (pGPIOx == GPIOJ) {
      GPIOJ_REG_RESET;
    } else if (pGPIOx == GPIOK) {
      GPIOK_REG_RESET;
    }
};

// Read and write data
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
    uint8_t value;
    value = (uint8_t) pGPIOx->IDR >> PinNumber & (0x00000001);
    return value;
};
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
    uint16_t value;
    value = (uint16_t) pGPIOx->IDR;
    return value;
};
void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value){
    if(Value == GPIO_PIN_SET){
        // write 1 to the output data register of pin number 
        pGPIOx->ODR |= (1<< PinNumber);
    }
    else{
        pGPIOx->ODR &= ~(1<< PinNumber);
    }
};
void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
    pGPIOx->ODR |= Value;
};
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
    pGPIOx->ODR ^= (1<<PinNumber);
};

// IRQ configuration and  handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){};
void GPIOIRQHandling(uint8_t PinNumber);
