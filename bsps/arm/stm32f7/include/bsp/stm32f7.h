/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 * @ingroup stm32f7_reg
 * @brief Register definitions.
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

#ifndef LIBBSP_ARM_STM32F7_STM32F7_H
#define LIBBSP_ARM_STM32F7_STM32F7_H

#include <bsp/utility.h>
#include <bspopts.h>
#include <bsp/irq.h>

#define STM32F7_BASE 0x00

#define __FPU_PRESENT 1U //for compiling FPU instructions
#define __FPU_USED 1U
#define __NVIC_PRIO_BITS 4U 


#ifdef STM32F7_FAMILY_F7XXXX

typedef enum
{
    /* ------------------- Cortex-M7 Processor Exceptions Numbers ------------------- */
    NonMaskableInt_IRQn         = -14, 
    MemoryManagement_IRQn       = -12, 
    BusFault_IRQn               = -11, 
    UsageFault_IRQn             = -10, 
    SVCall_IRQn                 = -5,  
    DebugMonitor_IRQn           = -4,  
    PendSV_IRQn                 = -2,  
    SysTick_IRQn                = -1,  
    /* ---------------------- STM32F7xx specific Interrupt Numbers ------------------ */
    WWDG_IRQn                   = STM32F7_IRQ_WWDG,  // 0
    PVD_IRQn                    = STM32F7_IRQ_PVD,   // 1
    TAMP_STAMP_IRQn             = STM32F7_IRQ_TAMP_STAMP, // 2
    RTC_WKUP_IRQn               = STM32F7_IRQ_RTC_WKUP,        // 3
    FLASH_IRQn                  = STM32F7_IRQ_FLASH,           // 4
    RCC_IRQn                    = STM32F7_IRQ_RCC,             // 5
    EXTI0_IRQn                  = STM32F7_IRQ_EXTI0,           // 6
    EXTI1_IRQn                  = STM32F7_IRQ_EXTI1,           // 7
    EXTI2_IRQn                  = STM32F7_IRQ_EXTI2,           // 8
    EXTI3_IRQn                  = STM32F7_IRQ_EXTI3,           // 9
    EXTI4_IRQn                  = STM32F7_IRQ_EXTI4,           // 10
    DMA1_Stream0_IRQn           = STM32F7_IRQ_DMA1_STREAM0,    // 11
    DMA1_Stream1_IRQn           = STM32F7_IRQ_DMA1_STREAM1,    // 12
    DMA1_Stream2_IRQn           = STM32F7_IRQ_DMA1_STREAM2,    // 13
    DMA1_Stream3_IRQn           = STM32F7_IRQ_DMA1_STREAM3,    // 14
    DMA1_Stream4_IRQn           = STM32F7_IRQ_DMA1_STREAM4,    // 15
    DMA1_Stream5_IRQn           = STM32F7_IRQ_DMA1_STREAM5,    // 16
    DMA1_Stream6_IRQn           = STM32F7_IRQ_DMA1_STREAM6,    // 17
    ADC_IRQn                    = STM32F7_IRQ_ADC,             // 18
    CAN1_TX_IRQn                = STM32F7_IRQ_CAN1_TX,         // 19
    CAN1_RX0_IRQn               = STM32F7_IRQ_CAN1_RX0,        // 20
    CAN1_RX1_IRQn               = STM32F7_IRQ_CAN1_RX1,        // 21
    CAN1_SCE_IRQn               = STM32F7_IRQ_CAN1_SCE,        // 22
    EXTI9_5_IRQn                = STM32F7_IRQ_EXTI9_5,         // 23
    TIM1_BRK_TIM9_IRQn          = STM32F7_IRQ_TIM1_BRK_TIM9,   // 24
    TIM1_UP_TIM10_IRQn          = STM32F7_IRQ_TIM1_UP_TIM10,   // 25
    TIM1_TRG_COM_TIM11_IRQn     = STM32F7_IRQ_TIM1_TRG_COM_TIM11, // 26
    TIM1_CC_IRQn                = STM32F7_IRQ_TIM1_CC,         // 27
    TIM2_IRQn                   = STM32F7_IRQ_TIM2,            // 28
    TIM3_IRQn                   = STM32F7_IRQ_TIM3,            // 29
    TIM4_IRQn                   = STM32F7_IRQ_TIM4,            // 30
    I2C1_EV_IRQn                = STM32F7_IRQ_I2C1_EV,         // 31
    I2C1_ER_IRQn                = STM32F7_IRQ_I2C1_ER,         // 32
    I2C2_EV_IRQn                = STM32F7_IRQ_I2C2_EV,         // 33
    I2C2_ER_IRQn                = STM32F7_IRQ_I2C2_ER,         // 34
    SPI1_IRQn                   = STM32F7_IRQ_SPI1,            // 35
    SPI2_IRQn                   = STM32F7_IRQ_SPI2,            // 36
    USART1_IRQn                 = STM32F7_IRQ_USART1,          // 37
    USART2_IRQn                 = STM32F7_IRQ_USART2,          // 38
    USART3_IRQn                 = STM32F7_IRQ_USART3,          // 39
    EXTI15_10_IRQn              = STM32F7_IRQ_EXTI15_10,       // 40
    RTC_ALARM_IRQn              = STM32F7_IRQ_RTC_ALARM,       // 41
    OTG_FS_WKUP_IRQn            = STM32F7_IRQ_OTG_FS_WKUP,     // 42
    TIM8_BRK_TIM12_IRQn         = STM32F7_IRQ_TIM8_BRK_TIM12,  // 43
    TIM8_UP_TIM13_IRQn          = STM32F7_IRQ_TIM8_UP_TIM13,   // 44
    TIM8_TRG_COM_TIM14_IRQn     = STM32F7_IRQ_TIM8_TRG_COM_TIM14, // 45
    TIM8_CC_IRQn                = STM32F7_IRQ_TIM8_CC,         // 46
    DMA1_Stream7_IRQn           = STM32F7_IRQ_DMA1_STREAM7,    // 47
    FSMC_IRQn                   = STM32F7_IRQ_FSMC,            // 48
    SDIO_IRQn                   = STM32F7_IRQ_SDIO,            // 49
    TIM5_IRQn                   = STM32F7_IRQ_TIM5,            // 50
    SPI3_IRQn                   = STM32F7_IRQ_SPI3,            // 51
    UART4_IRQn                  = STM32F7_IRQ_UART4,           // 52
    UART5_IRQn                  = STM32F7_IRQ_UART5,           // 53
    TIM6_DAC_IRQn               = STM32F7_IRQ_TIM6_DAC,        // 54
    TIM7_IRQn                   = STM32F7_IRQ_TIM7,            // 55
    DMA2_Stream0_IRQn           = STM32F7_IRQ_DMA2_STREAM0,    // 56
    DMA2_Stream1_IRQn           = STM32F7_IRQ_DMA2_STREAM1,    // 57
    DMA2_Stream2_IRQn           = STM32F7_IRQ_DMA2_STREAM2,    // 58
    DMA2_Stream3_IRQn           = STM32F7_IRQ_DMA2_STREAM3,    // 59
    DMA2_Stream4_IRQn           = STM32F7_IRQ_DMA2_STREAM4,    // 60
    ETH_IRQn                    = STM32F7_IRQ_ETH,             // 61
    ETH_WKUP_IRQn               = STM32F7_IRQ_ETH_WKUP,        // 62
    CAN2_TX_IRQn                = STM32F7_IRQ_CAN2_TX,         // 63
    CAN2_RX0_IRQn               = STM32F7_IRQ_CAN2_RX0,        // 64
    CAN2_RX1_IRQn               = STM32F7_IRQ_CAN2_RX1,        // 65
    CAN2_SCE_IRQn               = STM32F7_IRQ_CAN2_SCE,        // 66
    OTG_FS_IRQn                 = STM32F7_IRQ_OTG_FS,          // 67
    DMA2_Stream5_IRQn           = STM32F7_IRQ_DMA2_STREAM5,    // 68
    DMA2_Stream6_IRQn           = STM32F7_IRQ_DMA2_STREAM6,    // 69
    DMA2_Stream7_IRQn           = STM32F7_IRQ_DMA2_STREAM7,    // 70
    USART6_IRQn                 = STM32F7_IRQ_USART6,          // 71
    I2C3_EV_IRQn                = STM32F7_IRQ_I2C3_EV,         // 72
    I2C3_ER_IRQn                = STM32F7_IRQ_I2C3_ER,         // 73
    OTG_HS_EP1_OUT_IRQn         = STM32F7_IRQ_OTG_HS_EP1_OUT,  // 74
    OTG_HS_EP1_IN_IRQn          = STM32F7_IRQ_OTG_HS_EP1_IN,   // 75
    OTG_HS_WKUP_IRQn            = STM32F7_IRQ_OTG_HS_WKUP,     // 76
    OTG_HS_IRQn                 = STM32F7_IRQ_OTG_HS,          // 77
    DCMI_IRQn                   = STM32F7_IRQ_DCMI,            // 78
    CRYP_IRQn                   = STM32F7_IRQ_CRYP,            // 79
    HASH_RNG_IRQn               = STM32F7_IRQ_HASH_RNG,        // 80
    FPU_IRQn                    = STM32F7_IRQ_FPU    // 81
} IRQn_Type;

/**
 * @defgroup stm32f7_reg Register Defintions
 * @ingroup RTEMSBSPsARMSTM32F7
 * @brief Register Definitions
 * @{
 */

#define STM32F7_APB1_BASE (STM32F7_BASE + 0x40000000)
#define STM32F7_APB2_BASE (STM32F7_BASE + 0x40010000)
#define STM32F7_AHB1_BASE (STM32F7_BASE + 0x40020000)
#define STM32F7_AHB2_BASE (STM32F7_BASE + 0x50000000)

/**
 * @name STM32f7XXXX GPIO
 * @{
 */

#include <bsp/stm32f7xxxx_gpio.h>
#define STM32F7_GPIO_BASE (STM32F7_BASE + 0x40020000)
//#define STM32F7_GPIO(i) ((volatile stm32f7_gpio *) (STM32F7_GPIO_BASE) + (i))
#define STM32F7_GPIO(i) ((volatile stm32f7_gpio *) (STM32F7_GPIO_BASE + ((i) * 0x400))) //fixed offset of 0x400 byte

#define STM32F7_GPIOA_BASE STM32F7_GPIO_BASE      // 0x40020000
#define STM32F7_GPIOB_BASE (STM32F7_GPIO_BASE + 0x400) // 0x40020400
#define STM32F7_GPIOC_BASE (STM32F7_GPIO_BASE + 0x800) // 0x40020800
#define STM32F7_GPIOD_BASE (STM32F7_GPIO_BASE + 0xC00)

/** @} */

/**
 * @name STM32F7XXXX RCC
 * @{
 */

#include <bsp/stm32f7xxxx_rcc.h>
#define STM32F7_RCC ((volatile stm32f7_rcc *) (STM32F7_AHB1_BASE + 0x3800))

/** @} */

#include <bsp/stm32_i2c.h>

/**
 * @name STM32 I2C
 * @{
 */

#define STM32F7_I2C3 ((volatile stm32f7_i2c *) (STM32F7_BASE + 0x40005C00))
#define STM32F7_I2C2 ((volatile stm32f7_i2c *) (STM32F7_BASE + 0x40005800))
#define STM32F7_I2C1 ((volatile stm32f7_i2c *) (STM32F7_BASE + 0x40005400))

/** @} */

/**
 * @name STM32 USART
 * @{
 */

#include <bsp/stm32_usart.h>
#define STM32F7_USART_1 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40011000))
#define STM32F7_USART_2 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40004400))
#define STM32F7_USART_3 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40004800))
#define STM32F7_USART_4 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40004c00))
#define STM32F7_USART_5 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40005000))
#define STM32F7_USART_6 ((volatile stm32f7_usart *) (STM32F7_BASE + 0x40011400))

/** @} */

/**
 * @name STM32f7XXXX PWR
 * @{
 */

#include <bsp/stm32f7xxxx_pwr.h>
#define STM32F7_PWR ((volatile stm32f7_pwr *) (STM32F7_APB1_BASE + 0x7000))

/** @} */

/**
 * @name STM32f7XXXX EXTI
 * @{
 */

#include <bsp/stm32f7xxxx_exti.h>
#define STM32F7_EXTI ((volatile stm32f7_exti *) (STM32F7_APB2_BASE + 0x3c00))

/** @} */

/**
 * @name STM32f7XXXX SYSCFG
 * @{
 */

#include <bsp/stm32f7xxxx_syscfg.h>
#define STM32F7_SYSCFG ((volatile stm32f7_syscfg *) (STM32F7_APB2_BASE + 0x3800))

/** @} */

/**
 * @name STM32f7XXXX FLASH
 * @{
 */

#include <bsp/stm32f7xxxx_flash.h>
#define STM32F7_FLASH ((volatile stm32f7_flash *) (STM32F7_AHB1_BASE + 0x3c00))

/** @} */

/**
 * @name STM32f7XXXX TIM
 * @{
 */

#include <bsp/stm32f7xxxx_tim.h>
#define STM32F7_TIM1 ((volatile stm32f7_tim *) (STM32F7_APB2_BASE + 0x0000))
#define STM32F7_TIM2 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x0000))
#define STM32F7_TIM3 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x0400))
#define STM32F7_TIM4 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x0800))
#define STM32F7_TIM5 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x0c00))
#define STM32F7_TIM6 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x1000))
#define STM32F7_TIM7 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x1400))
#define STM32F7_TIM8 ((volatile stm32f7_tim *) (STM32F7_APB2_BASE + 0x0400))
#define STM32F7_TIM9 ((volatile stm32f7_tim *) (STM32F7_APB2_BASE + 0x4000))
#define STM32F7_TIM10 ((volatile stm32f7_tim *) (STM32F7_APB2_BASE + 0x4400))
#define STM32F7_TIM11 ((volatile stm32f7_tim *) (STM32F7_APB2_BASE + 0x4800))
#define STM32F7_TIM12 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x1800))
#define STM32F7_TIM13 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x1c00))
#define STM32F7_TIM14 ((volatile stm32f7_tim *) (STM32F7_APB1_BASE + 0x2000))

/** @} */

/**
 * @name STM32f7XXXX ADC
 * @{
 */

#include <bsp/stm32f7xxxx_adc.h>
#define STM32F7_ADC1 ((volatile stm32f7_adc_chan *) (STM32F7_APB2_BASE + 0x2000))
#define STM32F7_ADC2 ((volatile stm32f7_adc_chan *) (STM32F7_APB2_BASE + 0x2100))
#define STM32F7_ADC3 ((volatile stm32f7_adc_chan *) (STM32F7_APB2_BASE + 0x2200))
#define STM32F7_ADC_COMMON ((volatile stm32f7_adc_com *) (STM32F7_APB2_BASE + 0x2300))

/** @} */

/**
 * @name STM32f7XXXX OTGFS
 * @{
 */

#include <bsp/stm32f7xxxx_otgfs.h>
#define STM32F7_OTGFS_BASE (STM32F7_AHB2_BASE + 0x0000)
#define STM32F7_OTGFS_CORE ((volatile stm32f7_otgfs *) (STM32F7_OTGFS_BASE + 0x000))
#define STM32F7_OTGFS_DEV ((volatile stm32f7_otgfs_dregs *) (STM32F7_OTGFS_BASE + 0x800))
#define STM32F7_OTGFS_INEP ((volatile stm32f7_otgfs_inepregs *) (STM32F7_OTGFS_BASE + 0x900))
#define STM32F7_OTGFS_OUTEP ((volatile stm32f7_otgfs_outepregs *) (STM32F7_OTGFS_BASE + 0xb00))
#define STM32F7_OTGFS_PWRCTL ((volatile stm32f7_otgfs_pwrctlregs *) (STM32F7_OTGFS_BASE + 0xe00))

#define STM32F7_OTGFS_FIFO_BASE (STM32F7_OTGFS_BASE + USB_FIFO_BASE)

/** @} */

#endif /* STM32F7_FAMILY_F7XXXX */



#endif /* LIBBSP_ARM_STM32F7_STM32F7_H */
