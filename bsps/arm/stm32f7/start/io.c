/* SPDX-License-Identifier: BSD-2-Clause */

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

#include <bsp/io.h>
#include <bsp/rcc.h>
#include <bsp/stm32f7.h>

#include <rtems.h>

RTEMS_STATIC_ASSERT(sizeof(stm32f7_gpio_config) == 4, size_of_config);

void stm32f7_gpio_set_clock(int pin, bool set)
{
  int port = STM32F7_GPIO_PORT_OF_PIN(pin);
  stm32f7_rcc_index index = STM32F7_RCC_GPIOA + port;

  stm32f7_rcc_set_clock(index, set);
}

static void clear_and_set(
  volatile uint32_t *reg,
  unsigned index,
  unsigned width,
  uint32_t set
)
{
  uint32_t mask = (1U << width) - 1U;
  unsigned shift = width * index;
  uint32_t val = *reg;

  val &= ~(mask << shift);
  val |= set << shift;

  *reg = val;
}


static void set_config(unsigned pin, const stm32f7_gpio_config *config)
{
  unsigned port = STM32F7_GPIO_PORT_OF_PIN(pin);
  volatile stm32f7_gpio *gpio = STM32F7_GPIO(port);
  unsigned index = STM32F7_GPIO_INDEX_OF_PIN(pin);
  rtems_interrupt_level level;
  int set_or_clear_offset = config->fields.output ? 0 : 16;
#ifdef STM32F7_FAMILY_F7XXXX
  unsigned af_reg = index >> 3;
  unsigned af_index = index & 0x7;

  rtems_interrupt_disable(level);
  gpio->bsrr = 1U << (index + set_or_clear_offset);
  clear_and_set(&gpio->pupdr, index, 2, config->fields.pupd);
  clear_and_set(&gpio->otyper, index, 1, config->fields.otype);
  clear_and_set(&gpio->ospeedr, index, 2, config->fields.ospeed);
  clear_and_set(&gpio->afr [af_reg], af_index, 4, config->fields.af);
  clear_and_set(&gpio->moder, index, 2, config->fields.mode);
  rtems_interrupt_enable(level);

#endif /* STM32F7_FAMILY_F7XXXX */
}

void stm32f7_gpio_set_config(const stm32f7_gpio_config *config)
{
  int current = config->fields.pin_first;
  int last = config->fields.pin_last;

  while (current <= last) {
    stm32f7_gpio_set_clock(current, true);
    set_config(current, config);
    ++current;
  }
}

void stm32f7_gpio_set_config_array(const stm32f7_gpio_config *configs)
{
  stm32f7_gpio_config terminal = STM32F7_GPIO_CONFIG_TERMINAL;

  while (configs->value != terminal.value) {
    stm32f7_gpio_set_config(configs);
    ++configs;
  }
}

void stm32f7_gpio_set_output(int pin, bool set)
{
  int port = STM32F7_GPIO_PORT_OF_PIN(pin);
  volatile stm32f7_gpio *gpio = STM32F7_GPIO(port);
  int index = STM32F7_GPIO_INDEX_OF_PIN(pin);
  int set_or_clear_offset = set ? 0 : 16;

  gpio->bsrr = 1U << (index + set_or_clear_offset);
}

/*bool stm32f7_gpio_get_input(int pin)
{
  int port = STM32F7_GPIO_PORT_OF_PIN(pin);
  volatile stm32f7_gpio *gpio = STM32F7_GPIO(port);
  int index = STM32F7_GPIO_INDEX_OF_PIN(pin);

  return (gpio->idr & (1U << index)) != 0;
} */
