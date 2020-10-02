//*****************************************************************************
//
//! @file am_bsp_gpio.h
//!
//! @brief Functions to aid with configuring the GPIOs.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
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
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
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
#define AM_BSP_GPIO_BUTTON0             16
#define AM_BSP_GPIO_BUTTON1             18
#define AM_BSP_GPIO_BUTTON2             19
#define AM_BSP_GPIO_DISPLAY_CS          12
#define AM_BSP_GPIO_LED0                17
#define AM_BSP_GPIO_LED1                14
#define AM_BSP_GPIO_LED2                15
#define AM_BSP_GPIO_LED3                30
#define AM_BSP_GPIO_LED4                10

//*****************************************************************************
//
// BOOTLOADER_UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_BOOTLOADER_UART_RX  23
#define AM_BSP_GPIO_CFG_BOOTLOADER_UART_RX AM_HAL_PIN_23_UART0RX
#define AM_BSP_GPIO_BOOTLOADER_UART_TX  22
#define AM_BSP_GPIO_CFG_BOOTLOADER_UART_TX AM_HAL_PIN_22_UART0TX

//*****************************************************************************
//
// CLKOUT pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_CLKOUT_PIN          7
#define AM_BSP_GPIO_CFG_CLKOUT_PIN      AM_HAL_PIN_7_CLKOUT

//*****************************************************************************
//
// COM_UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_CTS        38
#define AM_BSP_GPIO_CFG_COM_UART_CTS    AM_HAL_PIN_38_UART0CTS
#define AM_BSP_GPIO_COM_UART_RTS        37
#define AM_BSP_GPIO_CFG_COM_UART_RTS    AM_HAL_PIN_37_UART0RTS
#define AM_BSP_GPIO_COM_UART_RX         23
#define AM_BSP_GPIO_CFG_COM_UART_RX     AM_HAL_PIN_23_UART0RX
#define AM_BSP_GPIO_COM_UART_TX         22
#define AM_BSP_GPIO_CFG_COM_UART_TX     AM_HAL_PIN_22_UART0TX

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
#define AM_BSP_GPIO_CFG_IOM0_SCK        (AM_HAL_PIN_5_M0SCK | AM_HAL_GPIO_INPEN | AM_HAL_GPIO_HIGH_DRIVE | AM_HAL_GPIO_24MHZ_ENABLE)
#define AM_BSP_GPIO_IOM0_SCL            5
#define AM_BSP_GPIO_CFG_IOM0_SCL        AM_HAL_PIN_5_M0SCL
#define AM_BSP_GPIO_IOM0_SDA            6
#define AM_BSP_GPIO_CFG_IOM0_SDA        AM_HAL_PIN_6_M0SDA

//*****************************************************************************
//
// IOM1 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_CS             12
#define AM_BSP_GPIO_CFG_IOM1_CS         AM_HAL_PIN_12_M1nCE0
#define AM_BSP_GPIO_IOM1_MISO           9
#define AM_BSP_GPIO_CFG_IOM1_MISO       AM_HAL_PIN_9_M1MISO
#define AM_BSP_GPIO_IOM1_MOSI           10
#define AM_BSP_GPIO_CFG_IOM1_MOSI       AM_HAL_PIN_10_M1MOSI
#define AM_BSP_GPIO_IOM1_SCK            8
#define AM_BSP_GPIO_CFG_IOM1_SCK        (AM_HAL_PIN_8_M1SCK | AM_HAL_GPIO_INPEN | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM1_SCL            8
#define AM_BSP_GPIO_CFG_IOM1_SCL        AM_HAL_PIN_8_M1SCL
#define AM_BSP_GPIO_IOM1_SDA            9
#define AM_BSP_GPIO_CFG_IOM1_SDA        AM_HAL_PIN_9_M1SDA

//*****************************************************************************
//
// IOM2 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_CS             26
#define AM_BSP_GPIO_CFG_IOM2_CS         AM_HAL_PIN_26_M2nCE0
#define AM_BSP_GPIO_IOM2_MISO           25
#define AM_BSP_GPIO_CFG_IOM2_MISO       AM_HAL_PIN_25_M2MISO
#define AM_BSP_GPIO_IOM2_MOSI           28
#define AM_BSP_GPIO_CFG_IOM2_MOSI       AM_HAL_PIN_28_M2MOSI
#define AM_BSP_GPIO_IOM2_SCK            27
#define AM_BSP_GPIO_CFG_IOM2_SCK        (AM_HAL_PIN_27_M2SCK | AM_HAL_GPIO_INPEN | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM2_SCL            27
#define AM_BSP_GPIO_CFG_IOM2_SCL        AM_HAL_PIN_27_M2SCL
#define AM_BSP_GPIO_IOM2_SDA            25
#define AM_BSP_GPIO_CFG_IOM2_SDA        AM_HAL_PIN_25_M2SDA

//*****************************************************************************
//
// IOM3 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_CS             26
#define AM_BSP_GPIO_CFG_IOM3_CS         AM_HAL_PIN_26_M3nCE0
#define AM_BSP_GPIO_IOM3_MISO           43
#define AM_BSP_GPIO_CFG_IOM3_MISO       AM_HAL_PIN_43_M3MISO
#define AM_BSP_GPIO_IOM3_MOSI           38
#define AM_BSP_GPIO_CFG_IOM3_MOSI       AM_HAL_PIN_38_M3MOSI
#define AM_BSP_GPIO_IOM3_SCK            42
#define AM_BSP_GPIO_CFG_IOM3_SCK        AM_HAL_PIN_42_M3SCK
#define AM_BSP_GPIO_IOM3_SCL            42
#define AM_BSP_GPIO_CFG_IOM3_SCL        AM_HAL_PIN_42_M3SCL
#define AM_BSP_GPIO_IOM3_SDA            43
#define AM_BSP_GPIO_CFG_IOM3_SDA        AM_HAL_PIN_43_M3SDA

//*****************************************************************************
//
// IOM4 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_CS             29
#define AM_BSP_GPIO_CFG_IOM4_CS         AM_HAL_PIN_29_M4nCE0
#define AM_BSP_GPIO_IOM4_MISO           40
#define AM_BSP_GPIO_CFG_IOM4_MISO       AM_HAL_PIN_40_M4MISO
#define AM_BSP_GPIO_IOM4_MOSI           44
#define AM_BSP_GPIO_CFG_IOM4_MOSI       AM_HAL_PIN_44_M4MOSI
#define AM_BSP_GPIO_IOM4_SCK            39
#define AM_BSP_GPIO_CFG_IOM4_SCK        (AM_HAL_PIN_39_M4SCK | AM_HAL_GPIO_24MHZ_ENABLE)
#define AM_BSP_GPIO_IOM4_SCL            39
#define AM_BSP_GPIO_CFG_IOM4_SCL        AM_HAL_PIN_39_M4SCL
#define AM_BSP_GPIO_IOM4_SDA            40
#define AM_BSP_GPIO_CFG_IOM4_SDA        AM_HAL_PIN_40_M4SDA

//*****************************************************************************
//
// IOM5 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_CS             24
#define AM_BSP_GPIO_CFG_IOM5_CS         AM_HAL_PIN_24_M5nCE0
#define AM_BSP_GPIO_IOM5_MISO           49
#define AM_BSP_GPIO_CFG_IOM5_MISO       AM_HAL_PIN_49_M5MISO
#define AM_BSP_GPIO_IOM5_MOSI           47
#define AM_BSP_GPIO_CFG_IOM5_MOSI       AM_HAL_PIN_47_M5MOSI
#define AM_BSP_GPIO_IOM5_SCK            48
#define AM_BSP_GPIO_CFG_IOM5_SCK        AM_HAL_PIN_48_M5SCK
#define AM_BSP_GPIO_IOM5_SCL            48
#define AM_BSP_GPIO_CFG_IOM5_SCL        AM_HAL_PIN_48_M5SCL
#define AM_BSP_GPIO_IOM5_SDA            49
#define AM_BSP_GPIO_CFG_IOM5_SDA        AM_HAL_PIN_49_M5SDA

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
// PDM pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_PDM_CLK             22
#define AM_BSP_GPIO_CFG_PDM_CLK         (AM_HAL_PIN_22_PDM_CLK | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_PDM_DATA            23
#define AM_BSP_GPIO_CFG_PDM_DATA        AM_HAL_PIN_23_PDM_DATA

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
