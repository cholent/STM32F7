/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 * @ingroup stm32f7_io
 * @brief IO support.
 */

/*
 * Copyright (C) 2012 Sebastian Huber
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIBBSP_ARM_STM32F7_IO_H
#define LIBBSP_ARM_STM32F7_IO_H

#include <stdbool.h>
#include <stdint.h>
#include <bspopts.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup stm32f7_io IO Support
 * @ingroup RTEMSBSPsARMSTM32F7
 * @brief IO Support
 * @{
 */

#define STM32F7_GPIO_PIN(port, index) ((((port) << 4) | (index)) & 0xff)

#define STM32F7_GPIO_PORT_OF_PIN(pin) (((pin) >> 4) & 0xf)

#define STM32F7_GPIO_INDEX_OF_PIN(pin) ((pin) & 0xf)

#ifdef STM32F7_FAMILY_F7XXXX

/**
 * @name Family F7XXXX
 * @{
 */

typedef enum {
  STM32F7_GPIO_MODE_INPUT,
  STM32F7_GPIO_MODE_OUTPUT,
  STM32F7_GPIO_MODE_AF,
  STM32F7_GPIO_MODE_ANALOG
} stm32f7_gpio_mode;

typedef enum {
  STM32F7_GPIO_OTYPE_PUSH_PULL,
  STM32F7_GPIO_OTYPE_OPEN_DRAIN
} stm32f7_gpio_otype;

typedef enum {
  STM32F7_GPIO_OSPEED_2_MHZ,
  STM32F7_GPIO_OSPEED_25_MHZ,
  STM32F7_GPIO_OSPEED_50_MHZ,
  STM32F7_GPIO_OSPEED_100_MHZ
} stm32f7_gpio_ospeed;

typedef enum {
  STM32F7_GPIO_NO_PULL,
  STM32F7_GPIO_PULL_UP,
  STM32F7_GPIO_PULL_DOWN
} stm32f7_gpio_pull;

typedef enum {
  STM32F7_GPIO_AF_SYSTEM = 0,
  STM32F7_GPIO_AF_TIM1 = 1,
  STM32F7_GPIO_AF_TIM2 = 1,
  STM32F7_GPIO_AF_TIM3 = 2,
  STM32F7_GPIO_AF_TIM4 = 2,
  STM32F7_GPIO_AF_TIM5 = 2,
  STM32F7_GPIO_AF_TIM8 = 3,
  STM32F7_GPIO_AF_TIM9 = 3,
  STM32F7_GPIO_AF_TIM10 = 3,
  STM32F7_GPIO_AF_TIM11 = 3,
  STM32F7_GPIO_AF_I2C1 = 4,
  STM32F7_GPIO_AF_I2C2 = 4,
  STM32F7_GPIO_AF_I2C3 = 4,
  STM32F7_GPIO_AF_SPI1 = 5,
  STM32F7_GPIO_AF_SPI2 = 5,
  STM32F7_GPIO_AF_SPI3 = 6,
  STM32F7_GPIO_AF_USART1 = 7,
  STM32F7_GPIO_AF_USART2 = 7,
  STM32F7_GPIO_AF_USART3 = 7,
  STM32F7_GPIO_AF_UART4 = 8,
  STM32F7_GPIO_AF_UART5 = 8,
  STM32F7_GPIO_AF_USART6 = 8,
  STM32F7_GPIO_AF_CAN1 = 9,
  STM32F7_GPIO_AF_CAN2 = 9,
  STM32F7_GPIO_AF_TIM12 = 9,
  STM32F7_GPIO_AF_TIM13 = 9,
  STM32F7_GPIO_AF_TIM14 = 9,
  STM32F7_GPIO_AF_OTG_FS = 10,
  STM32F7_GPIO_AF_OTG_HS = 10,
  STM32F7_GPIO_AF_ETH = 11,
  STM32F7_GPIO_AF_FSMC = 12,
  STM32F7_GPIO_AF_OTG_HS_FS = 12,
  STM32F7_GPIO_AF_SDIO = 12,
  STM32F7_GPIO_AF_DCMI = 13,
  STM32F7_GPIO_AF_EVENTOUT = 15
} stm32f7_gpio_af;


#define STM32F7_GPIO_PORT_B 1 
 

#define LED1_PIN STM32F7_GPIO_PIN(STM32F7_GPIO_PORT_B, 0)
#define LED2_PIN STM32F7_GPIO_PIN(STM32F7_GPIO_PORT_B, 7)
#define LED3_PIN STM32F7_GPIO_PIN(STM32F7_GPIO_PORT_B, 14)



typedef union {
  struct {
    uint32_t pin_first : 8;
    uint32_t pin_last : 8;
    uint32_t mode : 2;
    uint32_t otype : 1;
    uint32_t ospeed : 2;
    uint32_t pupd : 2;
    uint32_t output : 1;
    uint32_t af : 4;
    uint32_t reserved : 4;
  } fields;

  uint32_t value;
} stm32f7_gpio_config;


#define STM32F7_PIN_LED1 \
{ \
  .fields = { \
    .pin_first = LED1_PIN, \
    .pin_last = LED1_PIN, \
    .mode = STM32F7_GPIO_MODE_OUTPUT, \
    .otype = STM32F7_GPIO_OTYPE_PUSH_PULL, \
    .ospeed = STM32F7_GPIO_OSPEED_2_MHZ, \
    .pupd = STM32F7_GPIO_NO_PULL, \
    .output = 0, \
    .af = STM32F7_GPIO_AF_SYSTEM \
  } \
} 

#define STM32F7_PIN_LED2 \
{ \
  .fields = { \
    .pin_first = LED2_PIN, \
    .pin_last = LED2_PIN, \
    .mode = STM32F7_GPIO_MODE_OUTPUT, \
    .otype = STM32F7_GPIO_OTYPE_PUSH_PULL, \
    .ospeed = STM32F7_GPIO_OSPEED_2_MHZ, \
    .pupd = STM32F7_GPIO_NO_PULL, \
    .output = 0, \
    .af = STM32F7_GPIO_AF_SYSTEM \
  } \
}

#define STM32F7_PIN_LED3 \
{ \
  .fields = { \
    .pin_first = LED3_PIN, \
    .pin_last = LED3_PIN, \
    .mode = STM32F7_GPIO_MODE_OUTPUT, \
    .otype = STM32F7_GPIO_OTYPE_PUSH_PULL, \
    .ospeed = STM32F7_GPIO_OSPEED_2_MHZ, \
    .pupd = STM32F7_GPIO_NO_PULL, \
    .output = 0, \
    .af = STM32F7_GPIO_AF_SYSTEM \
  } \
} 


#define STM32F7_GPIO_CONFIG_TERMINAL \
  { { 0xff, 0xff, 0x3, 0x1, 0x3, 0x3, 0x1, 0xf, 0xf } }

/** @} */

#endif /* STM32F7_FAMILY_F7XXXX */



extern const stm32f7_gpio_config stm32f7_start_config_gpio [];

void stm32f7_gpio_set_clock(int pin, bool set);

void stm32f7_gpio_set_config(const stm32f7_gpio_config *config);

/**
 * @brief Sets the GPIO configuration of an array terminated by
 * STM32F7_GPIO_CONFIG_TERMINAL.
 */
void stm32f7_gpio_set_config_array(const stm32f7_gpio_config *configs);

void stm32f7_gpio_set_output(int pin, bool set);

bool stm32f7_gpio_get_input(int pin);

#ifdef STM32F7_FAMILY_F7XXXX

/**
 * @name Family F7XXXX
 * @{
 */

#define STM32F7_PIN_USART(port, idx, altfunc) \
  { \
    { \
      .pin_first = STM32F7_GPIO_PIN(port, idx), \
      .pin_last = STM32F7_GPIO_PIN(port, idx), \
      .mode = STM32F7_GPIO_MODE_AF, \
      .otype = STM32F7_GPIO_OTYPE_PUSH_PULL, \
      .ospeed = STM32F7_GPIO_OSPEED_2_MHZ, \
      .pupd = STM32F7_GPIO_PULL_UP, \
      .af = altfunc \
    } \
  }

#define STM32F7_PIN_USART1_TX_PA9 STM32F7_PIN_USART(0, 9, STM32F7_GPIO_AF_USART1)
#define STM32F7_PIN_USART1_TX_PB6 STM32F7_PIN_USART(1, 6, STM32F7_GPIO_AF_USART1)
#define STM32F7_PIN_USART1_RX_PA10 STM32F7_PIN_USART(0, 10, STM32F7_GPIO_AF_USART1)
#define STM32F7_PIN_USART1_RX_PB7 STM32F7_PIN_USART(1, 7, STM32F7_GPIO_AF_USART1)

#define STM32F7_PIN_USART2_TX_PA2 STM32F7_PIN_USART(0, 2, STM32F7_GPIO_AF_USART2)
#define STM32F7_PIN_USART2_TX_PD5 STM32F7_PIN_USART(3, 5, STM32F7_GPIO_AF_USART2)
#define STM32F7_PIN_USART2_RX_PA3 STM32F7_PIN_USART(0, 3, STM32F7_GPIO_AF_USART2)
#define STM32F7_PIN_USART2_RX_PD6 STM32F7_PIN_USART(3, 6, STM32F7_GPIO_AF_USART2)

#define STM32F7_PIN_USART3_TX_PC10 STM32F7_PIN_USART(2, 10, STM32F7_GPIO_AF_USART3)
#define STM32F7_PIN_USART3_TX_PD8 STM32F7_PIN_USART(3, 8, STM32F7_GPIO_AF_USART3)
#define STM32F7_PIN_USART3_RX_PC11 STM32F7_PIN_USART(2, 11, STM32F7_GPIO_AF_USART3)
#define STM32F7_PIN_USART3_RX_PD9 STM32F7_PIN_USART(3, 9, STM32F7_GPIO_AF_USART3)

#define STM32F7_PIN_UART4_TX_PA0 STM32F7_PIN_USART(0, 0, STM32F7_GPIO_AF_UART4)
#define STM32F7_PIN_UART4_TX_PC10 STM32F7_PIN_USART(2, 10, STM32F7_GPIO_AF_UART4)
#define STM32F7_PIN_UART4_RX_PA1 STM32F7_PIN_USART(0, 1, STM32F7_GPIO_AF_UART4)
#define STM32F7_PIN_UART4_RX_PC11 STM32F7_PIN_USART(2, 11, STM32F7_GPIO_AF_UART4)

#define STM32F7_PIN_UART5_TX_PC12 STM32F7_PIN_USART(2, 12, STM32F7_GPIO_AF_UART5)
#define STM32F7_PIN_UART5_RX_PD2 STM32F7_PIN_USART(3, 2, STM32F7_GPIO_AF_UART5)

#define STM32F7_PIN_USART6_TX_PC6 STM32F7_PIN_USART(2, 6, STM32F7_GPIO_AF_USART6)
#define STM32F7_PIN_USART6_RX_PC7 STM32F7_PIN_USART(2, 7, STM32F7_GPIO_AF_USART6)

/** @} */

#endif /* STM32F7_FAMILY_F7XXXX */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_STM32F7_IO_H */
