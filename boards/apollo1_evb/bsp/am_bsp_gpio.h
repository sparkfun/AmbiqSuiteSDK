//*****************************************************************************
//
//! @file am_bsp_gpio.h
//!
//! @brief Functions to aid with configuring the GPIOs.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_BSP_GPIO_H
#define AM_BSP_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Miscellaneous pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_BUTTON0             17
#define AM_BSP_GPIO_BUTTON1             18
#define AM_BSP_GPIO_BUTTON2             19
#define AM_BSP_GPIO_IOS_IRQ             2
#define AM_BSP_GPIO_LED0                34
#define AM_BSP_GPIO_LED1                33
#define AM_BSP_GPIO_LED2                28
#define AM_BSP_GPIO_LED3                26
#define AM_BSP_GPIO_LED4                25

//*****************************************************************************
//
// BOOTLOADER_UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_BOOTLOADER_UART_RX  36
#define AM_BSP_GPIO_CFG_BOOTLOADER_UART_RX AM_HAL_PIN_36_UARTRX
#define AM_BSP_GPIO_BOOTLOADER_UART_TX  35
#define AM_BSP_GPIO_CFG_BOOTLOADER_UART_TX AM_HAL_PIN_35_UARTTX

//*****************************************************************************
//
// CLKOUT pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_CLKOUT_PIN          24
#define AM_BSP_GPIO_CFG_CLKOUT_PIN      AM_HAL_PIN_24_CLKOUT

//*****************************************************************************
//
// COM_UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_CTS        38
#define AM_BSP_GPIO_CFG_COM_UART_CTS    AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_COM_UART_RTS        37
#define AM_BSP_GPIO_CFG_COM_UART_RTS    AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_COM_UART_RX         36
#define AM_BSP_GPIO_CFG_COM_UART_RX     AM_HAL_PIN_36_UARTRX
#define AM_BSP_GPIO_COM_UART_TX         35
#define AM_BSP_GPIO_CFG_COM_UART_TX     AM_HAL_PIN_35_UARTTX

//*****************************************************************************
//
// CORE pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_CORE_SWDCK          20
#define AM_BSP_GPIO_CFG_CORE_SWDCK      AM_HAL_PIN_20_SWDCK
#define AM_BSP_GPIO_CORE_SWDIO          21
#define AM_BSP_GPIO_CFG_CORE_SWDIO      AM_HAL_PIN_21_SWDIO

//*****************************************************************************
//
// DISPLAY pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISPLAY_BACKLT      29
#define AM_BSP_GPIO_CFG_DISPLAY_BACKLT  AM_HAL_PIN_OUTPUT
#define AM_BSP_GPIO_DISPLAY_D_C         31
#define AM_BSP_GPIO_CFG_DISPLAY_D_C     AM_HAL_PIN_OUTPUT

//*****************************************************************************
//
// IOM0 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_CS             11
#define AM_BSP_GPIO_CFG_IOM0_CS         AM_HAL_PIN_11_M0nCE0
#define AM_BSP_GPIO_IOM0_MISO           6
#define AM_BSP_GPIO_CFG_IOM0_MISO       AM_HAL_PIN_6_M0MISO
#define AM_BSP_GPIO_IOM0_MOSI           7
#define AM_BSP_GPIO_CFG_IOM0_MOSI       AM_HAL_PIN_7_M0MOSI
#define AM_BSP_GPIO_IOM0_SCK            5
#define AM_BSP_GPIO_CFG_IOM0_SCK        (AM_HAL_PIN_5_M0SCK | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM0_SCL            5
#define AM_BSP_GPIO_CFG_IOM0_SCL        (AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULL1_5K)
#define AM_BSP_GPIO_IOM0_SDA            6
#define AM_BSP_GPIO_CFG_IOM0_SDA        (AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULL1_5K)

//*****************************************************************************
//
// IOM1 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_CS             12
#define AM_BSP_GPIO_CFG_IOM1_CS         AM_HAL_PIN_12_M1nCE0
#define AM_BSP_GPIO_IOM1_MISO           9
#define AM_BSP_GPIO_CFG_IOM1_MISO       AM_HAL_PIN_9_M1MISO
#define AM_BSP_GPIO_IOM1_MONODISPLAY_CS 12
#define AM_BSP_GPIO_CFG_IOM1_MONODISPLAY_CS AM_HAL_PIN_OUTPUT
#define AM_BSP_GPIO_IOM1_MOSI           10
#define AM_BSP_GPIO_CFG_IOM1_MOSI       AM_HAL_PIN_10_M1MOSI
#define AM_BSP_GPIO_IOM1_SCK            8
#define AM_BSP_GPIO_CFG_IOM1_SCK        (AM_HAL_PIN_8_M1SCK | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM1_SCL            8
#define AM_BSP_GPIO_CFG_IOM1_SCL        (AM_HAL_PIN_8_M1SCL | AM_HAL_GPIO_PULL1_5K)
#define AM_BSP_GPIO_IOM1_SDA            9
#define AM_BSP_GPIO_CFG_IOM1_SDA        (AM_HAL_PIN_9_M1SDA | AM_HAL_GPIO_PULL1_5K)

//*****************************************************************************
//
// IOS pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_INT             4
#define AM_BSP_GPIO_CFG_IOS_INT         AM_HAL_PIN_4_SLINT
#define AM_BSP_GPIO_IOS_MISO            1
#define AM_BSP_GPIO_CFG_IOS_MISO        AM_HAL_PIN_1_SLMISO
#define AM_BSP_GPIO_IOS_MOSI            2
#define AM_BSP_GPIO_CFG_IOS_MOSI        AM_HAL_PIN_2_SLMOSI
#define AM_BSP_GPIO_IOS_SCK             0
#define AM_BSP_GPIO_CFG_IOS_SCK         AM_HAL_PIN_0_SLSCK
#define AM_BSP_GPIO_IOS_SCL             0
#define AM_BSP_GPIO_CFG_IOS_SCL         AM_HAL_PIN_0_SLSCL
#define AM_BSP_GPIO_IOS_SDA             1
#define AM_BSP_GPIO_CFG_IOS_SDA         AM_HAL_PIN_1_SLSDA
#define AM_BSP_GPIO_IOS_nCE             3
#define AM_BSP_GPIO_CFG_IOS_nCE         AM_HAL_PIN_3_SLnCE

//*****************************************************************************
//
// ITM pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO             41
#define AM_BSP_GPIO_CFG_ITM_SWO         AM_HAL_PIN_41_SWO

//*****************************************************************************
//
// VCOMP pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_VCOMP_RXO           42
#define AM_BSP_GPIO_CFG_VCOMP_RXO       AM_HAL_PIN_OUTPUT

//*****************************************************************************
//
// Convenience macros for enabling and disabling pins by function.
//
//*****************************************************************************
#define am_bsp_pin_enable(name)                                               \
    am_hal_gpio_pin_config(AM_BSP_GPIO_ ## name, AM_BSP_GPIO_CFG_ ## name);

#define am_bsp_pin_disable(name)                                              \
    am_hal_gpio_pin_config(AM_BSP_GPIO_ ## name, AM_HAL_PIN_DISABLE);

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_GPIO_H
