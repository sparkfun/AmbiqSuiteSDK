//*****************************************************************************
//
//! @file deepsleep.c
//!
//! @brief Example demonstrating how to enter deepsleep.
//!
//! Purpose: This example configures the device to go into a deep sleep mode. Once in
//! sleep mode the device has no ability to wake up. This example is merely to
//! provide the opportunity to measure deepsleep current without interrupts
//! interfering with the measurement.
//!
//! The example begins by printing out a banner announcement message through
//! the UART, which is then completely disabled for the remainder of execution.
//!
//! Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
//! Please note that text end-of-line is a newline (LF) character only.
//! Therefore, the UART terminal must be set to simulate a CR/LF.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"


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
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Deepsleep Example\n");

    //
    // To minimize power during the run, disable the UART.
    //
    am_bsp_uart_printf_disable();

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();
    
    //
    // Initialize the BLE controller for low-power operation.
    //
#if defined(BLE_3P3V_SW_WORKAROUND)
    am_bsp_ble_3p3v_low_power_mode();
#endif
    

#ifdef AM_PART_APOLLO
    //
    // Power down all SRAM banks.
    //
    am_hal_mcuctrl_sram_power_set(AM_HAL_MCUCTRL_SRAM_POWER_DOWN_ALL,
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_ALL);
#endif // AM_PART_APOLLO

#ifdef AM_PART_APOLLO2
    //
    // Turn OFF Flash1
    //
    PWRCTRL->MEMEN_b.FLASH1 = 0;
    while ( PWRCTRL->PWRONSTATUS_b.PD_FLAM1 != 0);

    //
    // Power down SRAM
    //
    PWRCTRL->SRAMPWDINSLEEP_b.SRAMSLEEPPOWERDOWN = PWRCTRL_SRAMPWDINSLEEP_SRAMSLEEPPOWERDOWN_ALLBUTLOWER8K;
#endif // AM_PART_APOLLO2

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    //
    // Turn OFF unneeded flash
    //
    if ( am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_MIN) )
    {
        while(1);
    }

    // For optimal Deep Sleep current, configure cache to be powered-down in deepsleep:
    am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);

    //
    // Power down SRAM, only 32K SRAM retained
    //
    am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
    am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_32K_DTCM);
#endif // AM_PART_APOLLO3

    while (1)
    {
        //
        // Go to Deep Sleep and stay there.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
