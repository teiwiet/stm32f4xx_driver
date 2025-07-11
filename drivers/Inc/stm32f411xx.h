#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_
#include <stdint.h>
#define __vo volatile
// base address
#define FLASH_BASEADDR 0x08000000U // flash base address page 75
#define SRAM1_BASEADDR 0x20000000U
// #define SRAM2_BASEADDR 0x2001
#define SRAM SRAM1_BASEADDR
#define ROM 0x1FFF0000U
#define PERISH_BASE 0x40000000U
#define APB1PERISH_BASE PERISH_BASE
#define APB2PERISH_BASE 0x40010000U
#define AHB1PERISH_BASE 0x40020000U
#define AHB2PERISH_BASE 0x50000000U
// AHB1 peripherals
#define GPIOA_BASEADDR (AHB1PERISH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERISH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERISH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERISH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERISH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERISH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERISH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERISH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERISH_BASE + 0x2000)
#define GPIOJ_BASEADDR (AHB1PERISH_BASE + 0x2400)
#define GPIOK_BASEADDR (AHB1PERISH_BASE + 0x2800)
#define RCC_BASEADDR (AHB1PERISH_BASE + 0x3800)
// APB1 peripherals
#define I2C1_BASEADDR (APB1PERISH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1PERISH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1PERISH_BASE + 0x5C00)

#define SPI2_BASEADDR (APB1PERISH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1PERISH_BASE + 0x3C00)

#define USART2_BASEADDR (APB1PERISH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERISH_BASE + 0x4800)

#define UART4_BASEADDR (APB1PERISH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERISH_BASE + 0x5000)
// APB2 peripherals
#define SPI1_BASEADDR (APB2PERISH_BASE + 0x3000)
#define USART1_BASEADDR (APB2PERISH_BASE + 0x1400)
#define USART6_BASEADDR (APB2PERISH_BASE + 0x1000)

#define SYSCFG_BASEADDR (APB2PERISH_BASE + 0x3800)
#define EXTI (APB2PERSIH_BASE + 0x3C00)
typedef struct {
  volatile uint32_t MODER;   // GPIO port mode register offset: 0x00
  volatile uint32_t OTYPER;  // GPIO port output type register offset: 0x04
  volatile uint32_t OSPEEDR; // GPIO port output speed register ofset: 0x08
  volatile uint32_t PUPDR; // GPIO port pull-up/pull-down register offset: 0x0C
  volatile uint32_t IDR;   // GPIO port input data register offset: 0x10
  volatile uint32_t ODR;   // GPIO port output data register offset: 0x14
  volatile uint32_t BSRR;  // GPIO port bit set/reset register offset: 0x18
  volatile uint32_t LCKR; // GPIO port configuration lock register offset : 0x1C
  volatile uint32_t
      AFR[2]; // GPIO alternate function low and hight reigister 0x20-0x24
} GPIO_RegDef_t;

typedef struct {
  __vo uint32_t RCC_CR;         // 0x00
  __vo uint32_t RCC_PLLCFGR;    // 0x04
  __vo uint32_t RCC_CFGR;       // 0x08
  __vo uint32_t RCC_CIR;        // 0x0C
  __vo uint32_t RCC_AHB1RSTR;   // 0x10
  __vo uint32_t RCC_AHB2RSTR;   // 0x14
  __vo uint32_t RCC_AHB3RSTR;   // 0x18
  uint32_t reserved;            // 0x1C
  __vo uint32_t RCC_APB1RSTR;   // 0x20
  __vo uint32_t RCC_APB2RSTR;   // 0x24
  uint32_t reserved1[2];        // 0x28-0x2C
  __vo uint32_t RCC_AHB1ENR;    // 0x30
  __vo uint32_t RCC_AHB2ENR;    // 0x34
  __vo uint32_t RCC_AHB3ENR;    // 0x38
  uint32_t reserved2;           // 0x
  __vo uint32_t RCC_APB1ENR;    // 0x40
  __vo uint32_t RCC_APB2ENR;    // 0x44
  uint32_t reserved3[2];        // 0x48-0x4C
  __vo uint32_t RCC_AHB1LPENR;  // 0x50
  __vo uint32_t RCC_AHB2LPENR;  // 0x54
  __vo uint32_t RCC_AHB3LPENR;  // 0x58
  uint32_t reserved4;           // 0x5C
  __vo uint32_t RCC_APB1LPENR;  // 0x60
  __vo uint32_t RCC_APB2LPENR;  // 0x64
  uint32_t reserved5;           // 0x68 - 0x6C
  __vo uint32_t RCC_BDCR;       // 0x70
  __vo uint32_t RCC_CSR;        // 0x74
  uint32_t reserved6;           // 0x78 - 0x7C
  __vo uint32_t RCC_SSCGR;      // 0x80
  __vo uint32_t RCC_PLLI2SCFGR; // 0x84
  __vo uint32_t RCC_PLLSAICFGR; // 0x88
  __vo uint32_t RCC_DCKCFGR;    // 0x8C
} RCC_RegDef_t;

// peripherals definitions (typecasted base adddress to GPIO_RegDef_t)
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
// clock enable & disable marcro for GPIOx peripherals
#define GPIOA_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN (RCC->RCC_AHB1ENR |= (1 << 10))

#define GPIOA_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI (RCC->RCC_AHB1ENR &= ~(1 << 10))

// clock enable & disable marcro for I2Cx peripherals
#define I2C1_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 23))

#define I2C1_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 23))

// clock enable & disable marcro for SPIx peripherals
#define SPI1_PCLK_EN (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 15))

#define SPI1_PCLK_DI (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 15))
// clock enable & disable marcro for UART and USART
#define USART1_PCLK_EN (RCC->RCC_APB2ENR |= (1 << 4))
#define USART2_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN (RCC->RCC_APB1ENR |= (1 << 20))
#define USART6_PCLK_EN (RCC->RCC_APB2ENR |= (1 << 5))

#define USART1_PCLK_DI (RCC->RCC_APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI (RCC->RCC_APB1ENR &= (1 << 19))
#define UART5_PCLK_DI (RCC->RCC_APB1ENR &= (1 << 20))
#define USART6_PCLK_DI (RCC->RCC_APB2ENR &= (1 << 5))
// RESET GPIOx peripherals 
#define GPIOA_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<0));(RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<1));(RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<2));(RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<3));(RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<4));(RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<5));(RCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<6));(RCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<7));(RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<8));(RCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<9));(RCC->RCC_AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET  do{(RCC->RCC_AHB1RSTR |= (1<<10));(RCC->RCC_AHB1RSTR &= ~(1<<10));}while(0)



#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET ENABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#include "stm32f411xx_gpio_driver.h"
#endif /* INC_STM32F411XX_H_ */
