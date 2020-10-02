//*****************************************************************************
//
//! @file am_bsp.c
//!
//! @brief Top level functions for performing board initialization.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo2_evb_bsp BSP for the Apollo2 AMAPH1KK-KBR EVB Rev 1.1 board
//! @ingroup BSP
//! @{
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

#include "am_bsp.h"

//*****************************************************************************
//
// LEDs
//
//*****************************************************************************
am_devices_led_t am_bsp_psLEDs[AM_BSP_NUM_LEDS] =
{
    {AM_BSP_GPIO_LED0, AM_DEVICES_LED_ON_HIGH | AM_DEVICES_LED_POL_DIRECT_DRIVE_M},
    {AM_BSP_GPIO_LED1, AM_DEVICES_LED_ON_HIGH | AM_DEVICES_LED_POL_DIRECT_DRIVE_M},
    {AM_BSP_GPIO_LED2, AM_DEVICES_LED_ON_HIGH | AM_DEVICES_LED_POL_DIRECT_DRIVE_M},
    {AM_BSP_GPIO_LED3, AM_DEVICES_LED_ON_HIGH | AM_DEVICES_LED_POL_DIRECT_DRIVE_M},
    {AM_BSP_GPIO_LED4, AM_DEVICES_LED_ON_HIGH | AM_DEVICES_LED_POL_DIRECT_DRIVE_M},
};

//*****************************************************************************
//
// Buttons.
//
//*****************************************************************************
am_devices_button_t am_bsp_psButtons[AM_BSP_NUM_BUTTONS] =
{
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON0, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON1, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON2, AM_DEVICES_BUTTON_NORMAL_HIGH),
};

//*****************************************************************************
//
// Power tracking structures for IOM and UART
//
//*****************************************************************************
am_bsp_uart_pwrsave_t am_bsp_uart_pwrsave[AM_REG_UART_NUM_MODULES];
am_bsp_iom_pwrsave_t am_bsp_iom_pwrsave[AM_REG_IOMSTR_NUM_MODULES];

//*****************************************************************************
//
// IOM MISO Pin assignments.
//
//*****************************************************************************
//
// Table of SPI pins - used by am_bsp_iom_enable() to make sure the MISO input
// is enabled (for power optimization).
//
#define AM_BSP_GPIO_UNDEF       0xDEADBEEF
static uint32_t g_SPIpins[AM_REG_IOMSTR_NUM_MODULES][2] =
{
#ifdef AM_BSP_GPIO_IOM0_SCK
    {AM_BSP_GPIO_IOM0_MISO,  AM_BSP_GPIO_CFG_IOM0_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0},
#endif
#ifdef AM_BSP_GPIO_IOM1_SCK
    {AM_BSP_GPIO_IOM1_MISO,  AM_BSP_GPIO_CFG_IOM1_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0},
#endif
#ifdef AM_BSP_GPIO_IOM2_SCK
    {AM_BSP_GPIO_IOM2_MISO,  AM_BSP_GPIO_CFG_IOM2_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0},
#endif
#ifdef AM_BSP_GPIO_IOM3_SCK
    {AM_BSP_GPIO_IOM3_MISO,  AM_BSP_GPIO_CFG_IOM3_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0},
#endif
#ifdef AM_BSP_GPIO_IOM4_SCK
    {AM_BSP_GPIO_IOM4_MISO,  AM_BSP_GPIO_CFG_IOM4_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0},
#endif
#ifdef AM_BSP_GPIO_IOM5_SCK
    {AM_BSP_GPIO_IOM5_MISO,  AM_BSP_GPIO_CFG_IOM5_MISO},
#else
    {AM_BSP_GPIO_UNDEF, 0}
#endif
};

#if defined(AM_BSP_GPIO_IOM0_SCK) || defined(AM_BSP_GPIO_IOM1_SCK) || \
    defined(AM_BSP_GPIO_IOM2_SCK) || defined(AM_BSP_GPIO_IOM3_SCK) || \
    defined(AM_BSP_GPIO_IOM4_SCK) || defined(AM_BSP_GPIO_IOM5_SCK)
//*****************************************************************************
//
//! @brief Enable the necessary pins for the given IOM module.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_spi_pins_enable(uint32_t ui32Module)
{
    switch (ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCK
        case 0:
            am_bsp_pin_enable(IOM0_SCK);
            am_bsp_pin_enable(IOM0_MISO);
            am_bsp_pin_enable(IOM0_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCK
        case 1:
            am_bsp_pin_enable(IOM1_SCK);
            am_bsp_pin_enable(IOM1_MISO);
            am_bsp_pin_enable(IOM1_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM2_SCK
        case 2:
            am_bsp_pin_enable(IOM2_SCK);
            am_bsp_pin_enable(IOM2_MISO);
            am_bsp_pin_enable(IOM2_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM3_SCK
        case 3:
            am_bsp_pin_enable(IOM3_SCK);
            am_bsp_pin_enable(IOM3_MISO);
            am_bsp_pin_enable(IOM3_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM4_SCK
        case 4:
            am_bsp_pin_enable(IOM4_SCK);
            am_bsp_pin_enable(IOM4_MISO);
            am_bsp_pin_enable(IOM4_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM5_SCK
        case 5:
            am_bsp_pin_enable(IOM5_SCK);
            am_bsp_pin_enable(IOM5_MISO);
            am_bsp_pin_enable(IOM5_MOSI);
            break;
#endif

        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while (1);
    }
}

//*****************************************************************************
//
//! @brief Disable the necessary pins for the given IOM module.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_spi_pins_disable(uint32_t ui32Module)
{
    switch (ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCK
        case 0:
            am_bsp_pin_disable(IOM0_SCK);
            am_bsp_pin_disable(IOM0_MISO);
            am_bsp_pin_disable(IOM0_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCK
        case 1:
            am_bsp_pin_disable(IOM1_SCK);
            am_bsp_pin_disable(IOM1_MISO);
            am_bsp_pin_disable(IOM1_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM2_SCK
        case 2:
            am_bsp_pin_disable(IOM2_SCK);
            am_bsp_pin_disable(IOM2_MISO);
            am_bsp_pin_disable(IOM2_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM3_SCK
        case 3:
            am_bsp_pin_disable(IOM3_SCK);
            am_bsp_pin_disable(IOM3_MISO);
            am_bsp_pin_disable(IOM3_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM4_SCK
        case 4:
            am_bsp_pin_disable(IOM4_SCK);
            am_bsp_pin_disable(IOM4_MISO);
            am_bsp_pin_disable(IOM4_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM5_SCK
        case 5:
            am_bsp_pin_disable(IOM5_SCK);
            am_bsp_pin_disable(IOM5_MISO);
            am_bsp_pin_disable(IOM5_MOSI);
            break;
#endif

        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while (1);
    }
}
#endif // AM_BSP_GPIO_IOMx_SCK

//*****************************************************************************
//
//! @brief Enable the IOM & the MISO input in SPI mode if used, for the given IOM module
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses during operation of the IOM to conserve power.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_enable(uint32_t ui32Module)
{
    am_hal_iom_enable(ui32Module);

    //
    // Check that the MISO pin is defined in BSP and we are in SPI mode.
    //
    if ( (AM_BSP_GPIO_UNDEF != g_SPIpins[ui32Module][0] ) &&
         (1 == AM_BFRn(IOMSTR, ui32Module, CFG, IFCSEL) ) )
    {
        am_hal_gpio_pin_config(g_SPIpins[ui32Module][0],
                               g_SPIpins[ui32Module][1]);
    }
}

//*****************************************************************************
//
//! @brief Disable the IOM & the MISO input in SPI mode if used, for the given IOM module
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses during operation of the IOM to conserve power.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_disable(uint32_t ui32Module)
{
    //
    // Check to make sure that MISO is defined and is actually being used.
    //
    if ( (AM_BSP_GPIO_UNDEF != g_SPIpins[ui32Module][0] ) &&
         (1 == AM_BFRn(IOMSTR, ui32Module, CFG, IFCSEL) ) )
    {
        if ( am_hal_gpio_pin_config_read(g_SPIpins[ui32Module][0]) ==
             (g_SPIpins[ui32Module][1]) )
        {
            am_hal_gpio_pin_config(g_SPIpins[ui32Module][0],
                                   (g_SPIpins[ui32Module][1] &
                                    (~AM_HAL_PIN_DIR_INPUT)));
        }
    }
    am_hal_iom_disable(ui32Module);
}

#if defined(AM_BSP_GPIO_IOM0_SCL) || defined(AM_BSP_GPIO_IOM1_SCL) || \
    defined(AM_BSP_GPIO_IOM2_SCL) || defined(AM_BSP_GPIO_IOM3_SCL) || \
    defined(AM_BSP_GPIO_IOM4_SCL) || defined(AM_BSP_GPIO_IOM5_SCL)
//*****************************************************************************
//
//! @brief Enable the necessary pins for the given IOM module in I2C mode.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module to work in I2C mode.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_i2c_pins_enable(uint32_t ui32Module)
{
    switch (ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCL
        case 0:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM0_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM0_SDA);

            am_bsp_pin_enable(IOM0_SCL);
            am_bsp_pin_enable(IOM0_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCL
        case 1:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM1_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM1_SDA);

            am_bsp_pin_enable(IOM1_SCL);
            am_bsp_pin_enable(IOM1_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM2_SCL
        case 2:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM2_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM2_SDA);

            am_bsp_pin_enable(IOM2_SCL);
            am_bsp_pin_enable(IOM2_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM3_SCL
        case 3:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM3_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM3_SDA);

            am_bsp_pin_enable(IOM3_SCL);
            am_bsp_pin_enable(IOM3_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM4_SCL
        case 4:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM4_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM4_SDA);

            am_bsp_pin_enable(IOM4_SCL);
            am_bsp_pin_enable(IOM4_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM5_SCL
        case 5:
            //
            // Set pins high to prevent bus dips.
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM5_SCL);
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_IOM5_SDA);

            am_bsp_pin_enable(IOM5_SCL);
            am_bsp_pin_enable(IOM5_SDA);
            break;
#endif
        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while (1);
    }
}

//*****************************************************************************
//
//! @brief Disable the necessary pins for the given IOM module in I2C mode.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module to work in I2C mode.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_i2c_pins_disable(uint32_t ui32Module)
{
    switch (ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCL
        case 0:
            am_bsp_pin_disable(IOM0_SCL);
            am_bsp_pin_disable(IOM0_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCL
        case 1:
            am_bsp_pin_disable(IOM1_SCL);
            am_bsp_pin_disable(IOM1_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM2_SCL
        case 2:
            am_bsp_pin_disable(IOM2_SCL);
            am_bsp_pin_disable(IOM2_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM3_SCL
        case 3:
            am_bsp_pin_disable(IOM3_SCL);
            am_bsp_pin_disable(IOM3_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM4_SCL
        case 4:
            am_bsp_pin_disable(IOM4_SCL);
            am_bsp_pin_disable(IOM4_SDA);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM5_SCL
        case 5:
            am_bsp_pin_disable(IOM5_SCL);
            am_bsp_pin_disable(IOM5_SDA);
            break;
#endif
        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while (1);
    }
}
#endif // AM_BSP_GPIO_IOMx_SCL

//*****************************************************************************
//
//! @brief Prepare the MCU for low power operation.
//!
//! This function enables several power-saving features of the MCU, and
//! disables some of the less-frequently used peripherals. It also sets the
//! system clock to 24 MHz.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_low_power_init(void)
{
    //
    // Enable internal buck converters.
    //
    am_hal_pwrctrl_bucks_init();

    //
    // Initialize for low power in the power control block
    //
    am_hal_pwrctrl_low_power_init();

    //
    // Turn off the voltage comparator as this is enabled on reset.
    //
    am_hal_vcomp_disable();

    //
    // Run the RTC off the LFRC.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);

    //
    // Stop the XTAL.
    //
    am_hal_clkgen_osc_stop(AM_HAL_CLKGEN_OSC_XT);

    //
    // Disable the RTC.
    //
    am_hal_rtc_osc_disable();
}

//*****************************************************************************
//
// brief Determine the pin that a UART instance is using.
//
// This static, BSP-specific function determines which pin a particular
// UART instance is assigned to.  That is, it maps a UART module instance
// number to the pin used for TX.
//
// return The assigned pin number for the given UART module (instance).
//        0 if no pin assigned.
//
// This function must be customized for each BSP.
// To modify for your board:
//  - Determine the UART TX pin number used for each UART instance (module).
//  - Return that pin number for the appropriate instance in the switch stmt.
//
//*****************************************************************************
static uint32_t
bsp_uart_tx_pin_get(uint32_t ui32Module)
{
    switch ( ui32Module )
    {
        case 0:
            return 0;
        case 1:
            return AM_BSP_GPIO_BOOTLOADER_UART_TX;
        default:
            return 0;
    }
}

//*****************************************************************************
//
// brief Determine the CS pin that a IOM instance is using.
//
// This static, BSP-specific function determines which pin a particular
// IOM instance and CS is assigned to.  That is, it maps an IOM module instance
// number and CS to the pin used for that CS.
//
// This function must be customized for each BSP!
//
// return The assigned CS pin number for the given IOM module (instance).
//        0 if no pin assigned.
//
// This function must be customized for each BSP.
// To modify for your board:
//  - Determine the IOM SPI module and CE pin number (there are up to 8
//    CE per IOM module).
//  - Return that pin number for the appropriate instance/CE in the switch stmt.
//
//*****************************************************************************
static uint32_t
bsp_iom_cs_pin_get(uint32_t ui32Module, uint32_t ui32CS)
{
    switch ( ui32Module )
    {
        case 0:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM0_CS
                case 0:
                    return AM_BSP_GPIO_IOM0_CS;
#endif
                default:
                    return 0;
            }
        case 1:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM1_CS
                case 0:
                    return AM_BSP_GPIO_IOM1_CS;
#endif
                default:
                    return 0;
            }
        case 2:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM2_CS
                case 0:
                    return AM_BSP_GPIO_IOM2_CS;
#endif
                default:
                    return 0;
            }
        case 3:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM3_CS
                case 0:
                    return AM_BSP_GPIO_IOM3_CS;
#endif
                default:
                    return 0;
            }
        case 4:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM4_CS
                case 0:
                    return AM_BSP_GPIO_IOM4_CS;
#endif
                default:
                    return 0;
            }
        case 5:
            switch ( ui32CS )
            {
#ifdef AM_BSP_GPIO_IOM5_CS
                case 0:
                    return AM_BSP_GPIO_IOM5_CS;
#endif
                default:
                    return 0;
            }
        default:
            return 0;
    }
}

//*****************************************************************************
//
//! @brief Power down the UART, save its state, and maintain the pin level.
//!
//! This function powers down the given UART, saves its state, and reconfigures
//! the pin as GPIO and sets it to a high level.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_uart_power_off_save(uint32_t ui32Module)
{
    uint32_t ui32UartTxPin;

    if ( ui32Module >= AM_REG_UART_NUM_MODULES )
    {
        return;
    }

    //
    // Determine the pin number for this UART
    //
    ui32UartTxPin = bsp_uart_tx_pin_get(ui32Module);

    if ( ui32UartTxPin == 0 )
    {
        return;
    }

    //
    // Save the pin number
    //
    am_bsp_uart_pwrsave[ui32Module].ui32TxPinNum = ui32UartTxPin;

    //
    // Mark the save structure as valid.
    //
    am_bsp_uart_pwrsave[ui32Module].bSaved = true;

    //
    // We need to maintain a high level output on the TX pin.
    // First, save off the current pin configuration.
    //
    am_bsp_uart_pwrsave[ui32Module].ui32TxPinCfg =
        am_hal_gpio_pin_config_read(ui32UartTxPin);

    //
    // Enable the GPIO and set it high so that it's high when reconfigured.
    //
    am_hal_gpio_out_bit_set(ui32UartTxPin);

    //
    // Configure the pin as GPIO (push-pull) output.
    //
    am_hal_gpio_pin_config(ui32UartTxPin, AM_HAL_GPIO_OUTPUT);

    //
    // Now that the pin is high, call the HAL to save the current state of the
    // UART and power it down.
    //
    am_hal_uart_power_off_save(ui32Module);
}

//*****************************************************************************
//
//! @brief Restore and power up the UART.
//!
//! This function restores the state of the UART and powers it up.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_uart_power_on_restore(uint32_t ui32Module)
{
    uint32_t ui32UartTxPin;

    if ( ui32Module >= AM_REG_UART_NUM_MODULES )
    {
        return;
    }

    //
    // Make sure this restore is a companion to a previous save call.
    //
    if ( !am_bsp_uart_pwrsave[ui32Module].bSaved )
    {
        return;
    }

    ui32UartTxPin = bsp_uart_tx_pin_get(ui32Module);

    if ( ui32UartTxPin == 0 )
    {
        return;
    }

    //
    // Mark the save structure as invalid.
    //
    am_bsp_uart_pwrsave[ui32Module].bSaved = false;

    //
    // First have the HAL restore the UART to its previous state.
    //
    am_hal_uart_power_on_restore(ui32Module);

    //
    // Now re-configure the UART TX pin.
    //
    am_hal_gpio_pin_config(ui32UartTxPin,                           \
                           am_bsp_uart_pwrsave[ui32Module].ui32TxPinCfg);
}

//*****************************************************************************
//
//! @brief Power down the IOM, save its state, and maintain the pin level.
//!
//! This function powers down the given IOM, saves its state, and reconfigures
//! the pin as GPIO and sets it to a high level.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_power_off_save(uint32_t ui32Module)
{
    uint32_t ui32IomCsPin, ui32CS;

    if ( ui32Module >= AM_REG_IOMSTR_NUM_MODULES )
    {
        return;
    }

    //
    // Determine the pin number for this IOM and CS (each IOM can support
    // up to 8 SPI devices).
    //
    for ( ui32CS = 0; ui32CS < 8; ui32CS++ )
    {
        ui32IomCsPin = bsp_iom_cs_pin_get(ui32Module, ui32CS);

        //
        // Save the pin number.
        //
        am_bsp_iom_pwrsave[ui32Module].ui32CsPinNum[ui32CS] = ui32IomCsPin;

        if ( ui32IomCsPin == 0 )
        {
            //
            // Mark the structure as false
            //
            am_bsp_iom_pwrsave[ui32Module].bSaved[ui32CS] = false;
            continue;
        }

        //
        // We have a CS pin that needs to be taken high.
        // Mark the save structure as valid.
        //
        am_bsp_iom_pwrsave[ui32Module].bSaved[ui32CS] = true;

        //
        // We need to maintain a high level output on the TX pin.
        // First, save off the current pin configuration.
        //
        am_bsp_iom_pwrsave[ui32Module].ui32CsPinCfg[ui32CS] =
            am_hal_gpio_pin_config_read(ui32IomCsPin);

        //
        // Enable the GPIO and set it high so that it's high when reconfigured.
        //
        am_hal_gpio_out_bit_set(ui32IomCsPin);

        //
        // Configure the pin as GPIO (push-pull) output.
        //
        am_hal_gpio_pin_config(ui32IomCsPin, AM_HAL_GPIO_OUTPUT);
    }

    //
    // Now that all CS pins have been taken high, call the HAL to save the
    // current state of the IOM and power it down.
    //
    am_hal_iom_power_off_save(ui32Module);
}

//*****************************************************************************
//
//! @brief Restore and power up the IOM.
//!
//! This function restores the state of the IOM and powers it up.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_power_on_restore(uint32_t ui32Module)
{
    uint32_t ui32IomCsPin, ui32CS;

    if ( ui32Module >= AM_REG_IOMSTR_NUM_MODULES )
    {
        return;
    }

    //
    // First have the HAL restore the IOM to its previous state.
    //
    am_hal_iom_power_on_restore(ui32Module);

    //
    // Now set all valid CS pins to their original configuration.
    //
    for ( ui32CS = 0; ui32CS < 8; ui32CS++ )
    {
        if ( !am_bsp_iom_pwrsave[ui32Module].bSaved[ui32CS] )
        {
            continue;
        }

        //
        // Mark the structure as false.
        //
        am_bsp_iom_pwrsave[ui32Module].bSaved[ui32CS] = false;

        //
        // Get and validate the pin number
        //
        ui32IomCsPin = am_bsp_iom_pwrsave[ui32Module].ui32CsPinNum[ui32CS];
        if ( (ui32IomCsPin == 0)  ||  (ui32IomCsPin > 63) )
        {
            continue;
        }

        //
        // Now re-configure the IOM TX pin.
        //
        am_hal_gpio_pin_config(ui32IomCsPin,                            \
                               am_bsp_iom_pwrsave[ui32Module].ui32CsPinCfg[ui32CS]);
    }
}

//*****************************************************************************
//
//! @brief Enable the TPIU and ITM for debug printf messages.
//!
//! This function enables TPIU registers for debug printf messages and enables
//! ITM GPIO pin to SWO mode. This function should be called after reset and
//! after waking up from deep sleep.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_debug_printf_enable(void)
{
    am_hal_tpiu_config_t TPIUcfg;

    if ( g_ui32HALflags & AM_HAL_FLAGS_ITMSKIPENABLEDISABLE_M )
    {
        return;
    }

    //
    // Write to the ITM control and status register.
    //
    AM_REGVAL(AM_REG_ITM_TCR_O) =
        AM_WRITE_SM(AM_REG_ITM_TCR_ATB_ID, 0x15)      |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_FREQ, 1)        |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_PRESCALE, 1)    |
        AM_WRITE_SM(AM_REG_ITM_TCR_SWV_ENABLE, 1)     |
        AM_WRITE_SM(AM_REG_ITM_TCR_DWT_ENABLE, 0)     |
        AM_WRITE_SM(AM_REG_ITM_TCR_SYNC_ENABLE, 0)    |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_ENABLE, 0)      |
        AM_WRITE_SM(AM_REG_ITM_TCR_ITM_ENABLE, 1);

    //
    // Enable the ITM and TPIU
    //
    TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_1M;
    am_hal_tpiu_enable(&TPIUcfg);
    am_bsp_pin_enable(ITM_SWO);
}

//*****************************************************************************
//
//! @brief Enable the TPIU and ITM for debug printf messages.
//!
//! This function disables TPIU registers for debug printf messages and
//! enables ITM GPIO pin to GPIO mode and prepares the MCU to go to deep sleep.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_debug_printf_disable(void)
{
    if ( g_ui32HALflags & AM_HAL_FLAGS_ITMSKIPENABLEDISABLE_M )
    {
        return;
    }

    //
    // Disable the TPIU
    //
    am_hal_itm_not_busy();
    am_hal_gpio_pin_config(AM_BSP_GPIO_ITM_SWO, AM_HAL_GPIO_OUTPUT);
    am_hal_tpiu_disable();
}

//*****************************************************************************
//
//! @brief ITM-based string print function.
//!
//! This function is used for printing a string via the ITM.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_itm_string_print(char *pcString)
{
    am_hal_itm_print(pcString);
}

//*****************************************************************************
//
//! @brief UART-based string print function.
//!
//! This function is used for printing a string via the UART, which for some
//! MCU devices may be multi-module.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_uart_string_print(char *pcString)
{
    am_hal_uart_string_transmit_polled(AM_BSP_UART_PRINT_INST, pcString);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
