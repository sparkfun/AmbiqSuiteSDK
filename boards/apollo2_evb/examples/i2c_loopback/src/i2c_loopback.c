//*****************************************************************************
//
//! @file i2c_loopback.c
//!
//! @brief Example of I2C operation using IOM #0 talking to the IOS over I2C
//!
//! SWO is configured in 1M baud, 8-n-1 mode.
//! @verbatim
//! PIN Fly Lead Assumptions for the I/O Master (IOM):
//! IOM0
//!        GPIO[5] == IOM4 I2C SCL
//!        GPIO[6] == IOM4 I2C SDA
//!
//! IOS
//! PIN Fly Lead Assumptions for the I/O Slave (IOS):
//!        GPIO[0] == IOS I2C SCL
//!        GPIO[1] == IOS I2C SDA
//!
//! Connect IOM4 I2C pins to corresponding IOS I2C pins
//!
//! @endverbatim
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define IOM_MODULE     0 // This will have a side benefit of testing IOM4 in offset mode

// We need to shift one extra
#define GET_I2CADDR(cfg)    \
(((cfg) & AM_REG_IOSLAVE_CFG_I2CADDR_M) >> (AM_REG_IOSLAVE_CFG_I2CADDR_S + 1))

#define IOS_ADDRESS             0x20

//
// IOM Queue Memory
//
am_hal_iom_queue_entry_t g_psQueueMemory[32];

uint8_t g_testdata[] =
{
  0x15, 0x2f, 0x8f, 0xdf, 0xc3, 0xa0, 0x6c, 0x16, 0x19, 0xa5, 0x18, 0xc1, 0x18, 0x65, 0xa8, 0x44
};

uint8_t g_resultdata[16];


//*****************************************************************************
//
// I2C Slave Configuration
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSI2cConfig =
{
    // Configure the IOS in I2C mode.
    .ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(IOS_ADDRESS),

    // Eliminate the "read-only" section, so an external host can use the
    // entire "direct write" section.
    .ui32ROBase = 0x78,

    // Set the FIFO base to the maximum value, making the "direct write"
    // section as big as possible.
    .ui32FIFOBase = 0x80,

    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    .ui32RAMBase = 0x100,
    // FIFO Threshold - set to half the size
    .ui32FIFOThreshold = 0x40,
};

//*****************************************************************************
//
// I2C Master Configuration
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_I2CMODE,
    .ui32ClockFrequency = AM_HAL_IOM_1MHZ,
    .ui8WriteThreshold = 12,
    .ui8ReadThreshold = 120,
};

//
//! Take over default ISR for IOM 0. (Queue mode service)
//
void
am_iomaster0_isr(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_iom_int_status_get(IOM_MODULE, true);

    am_hal_iom_int_clear(IOM_MODULE, ui32Status);

    am_hal_iom_queue_service(IOM_MODULE, ui32Status);
}


//*****************************************************************************
//
// Start up the ITM interface.
//
//*****************************************************************************
void
itm_start(void)
{
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal.
    //
    am_util_stdio_terminal_clear();
}

//*****************************************************************************
//
// Configure the IOS as I2C slave.
//
//*****************************************************************************
static void
ios_set_up(void)
{
  // Configure I2C interface
    am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCL);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLSDA);

    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_config(&g_sIOSI2cConfig);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_access_int_clear(AM_HAL_IOS_ACCESS_INT_ALL);
    am_hal_ios_int_clear(AM_HAL_IOS_INT_ALL);

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSACC);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSLAVE);

    // Set up the IOSINT interrupt pin
    am_hal_gpio_pin_config(4, AM_HAL_PIN_4_SLINT);
}

//*****************************************************************************
//
// Configure the IOM as I2C master.
//
//*****************************************************************************
static void
iom_set_up(void)
{
    //
    // Enable power to IOM.
    //
    am_hal_iom_pwrctrl_enable(IOM_MODULE);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_config(IOM_MODULE, &g_sIOMI2cConfig);

    //
    // Set pins high to prevent bus dips.
    //
    am_hal_gpio_out_bit_set(5);
    am_hal_gpio_out_bit_set(6);

#ifdef INTERNAL_LOOPBACK
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCLLB | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_SLSDALB | AM_HAL_GPIO_PULLUP);
    AM_REG(GPIO, LOOPBACK) = IOM_MODULE;
#else
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCL | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA | AM_HAL_GPIO_PULLUP);
#endif

    am_hal_iom_int_enable(IOM_MODULE, 0xFF);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);

    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(IOM_MODULE);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    itm_start();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("I2C Loopback Example using IOMSTR #4 and IOSLAVE");

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // Enable Interrupts.
    //
    am_hal_interrupt_master_enable();

    //
    // Set up the IOS
    //
    ios_set_up();

    //
    // Set up the IOM
    //
    iom_set_up();

    //
    // Perform a 16-byte transfer as a blocking operation.
    //
    am_hal_iom_i2c_write(IOM_MODULE, GET_I2CADDR(g_sIOSI2cConfig.ui32InterfaceSelect),
                       (uint32_t *)&g_testdata, 16, AM_HAL_IOM_OFFSET(0x80));
    am_util_stdio_printf("\nBlocking Transfer Complete\n");

    for ( uint32_t i = 0; i < 16; i++ )
    {
        if ( g_testdata[i] != am_hal_ios_lram_read(i) )
        {
            am_util_stdio_printf("\nIOS Data did not match IOM data sent!!!\n");
            break;
        }
    }

    //
    // Set up the IOM transaction queue.
    //
    am_hal_iom_queue_init(IOM_MODULE, g_psQueueMemory, sizeof(g_psQueueMemory));

    //
    // Perform a 16-byte transfer as a blocking operation.
    //
    for ( uint32_t i = 0; i < sizeof(g_testdata); i++ )
    {
        am_hal_iom_i2c_write(IOM_MODULE, GET_I2CADDR(g_sIOSI2cConfig.ui32InterfaceSelect),
                             (uint32_t *)&g_testdata[i], 1, AM_HAL_IOM_OFFSET(0x80 | i));
    }

    am_util_delay_ms(10);
    am_util_stdio_printf("\nNon-Blocking Transfer Complete\n");

    for ( uint32_t i = 0; i < 16; i++ )
    {
        if ( g_testdata[i] != am_hal_ios_lram_read(i) )
        {
            am_util_stdio_printf("\nIOS Data did not match IOM data sent!!!\n");
            break;
        }
    }

    // Make sure the print is complete
    am_util_delay_ms(100);

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
