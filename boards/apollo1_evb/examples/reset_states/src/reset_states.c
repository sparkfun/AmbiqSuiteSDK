//*****************************************************************************
//
//! @file reset_states.c
//!
//! @brief Example of various reset options in Apollo.
//!
//! This example shows a simple configuration of the watchdog. It will print
//! a banner message, configure the watchdog for both interrupt and reset
//! generation, and immediately start the watchdog timer.
//! The watchdog ISR provided will 'pet' the watchdog four times, printing
//! a notification message from the ISR each time.
//! On the fifth interrupt, the watchdog will not be pet, so the 'reset'
//! action will eventually be allowed to occur.
//! On the sixth timeout event, the WDT should issue a system reset, and the
//! program should start over from the beginning.
//!
//! The program will repeat the following sequence on the console:
//! (POI Reset) 5 Interrupts - (WDT Reset) 3 Interrupts - (POR Reset) 3 Interrupts
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

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint8_t g_ui8NumWatchdogInterrupts = 0;

am_hal_wdt_config_t g_sWatchdogConfig =
{
    //
    // Select the Apollo 1 Clock Rate.
    //
    .ui32Config = AM_HAL_WDT_ENABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,

    //
    // Set WDT interrupt timeout for 3/4 second.
    //
    128 * 3 / 4,

    //
    // Set WDT reset timeout for 1.5 seconds.
    //
    128 * 3 / 2
};

enum
{
     NEXT_WATCHDOG = 0,
     NEXT_SWPOR,
     NEXT_SWPOI,
}g_eNextInterrupt = NEXT_WATCHDOG;

//*****************************************************************************
//
// Interrupt handler for the watchdog.
//
//*****************************************************************************
void
am_watchdog_isr(void)
{
    //
    // Clear the watchdog interrupt.
    //
    am_hal_wdt_int_clear();

    //
    // Catch the first four watchdog interrupts, but let the fifth through
    // untouched.
    //
    if (g_ui8NumWatchdogInterrupts < 4)
    {
        //
        // Restart the watchdog.
        //
        am_hal_wdt_restart();
    }

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Send a status message and give it some time to print.
    //
    am_util_stdio_printf("Interrupt!!\n");
    am_util_delay_ms(10);

    //
    // On the second interrupt do a different kind of reset
    //
    if (g_ui8NumWatchdogInterrupts == 2)
    {
        //
        //  If it's not a watch dog interrupt we will reset here.
        //
        if (g_eNextInterrupt != NEXT_WATCHDOG )
        {
            //
            // Stop the watchdog.
            //
            am_hal_wdt_halt();

            //
            // Check if we want to do a POR or POI.
            //
            if (g_eNextInterrupt == NEXT_SWPOR)
            {
                am_hal_reset_por();
            }
            else
            {
                am_hal_reset_poi();
            }
        }
    }

    //
    // Increment the number of watchdog interrupts.
    //
    g_ui8NumWatchdogInterrupts++;
}

//*****************************************************************************
//
// Function to decode and print the reset cause.
//
//*****************************************************************************
void
reset_decode(void)
{
    uint32_t ui32ResetStatus;

    //
    // Print out reset status register upon entry.
    //
    ui32ResetStatus = am_hal_reset_status_get();
    am_util_stdio_printf("Reset Status Register = 0x%02x\n", ui32ResetStatus);

    //
    // POWER CYCLE.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_POWER_CYCLE)
    {
        am_util_stdio_printf("Power Cycled\n");
        g_eNextInterrupt = NEXT_WATCHDOG;
    }

    //
    // WATCHDOG.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_WDT)
    {
        am_util_stdio_printf("Watchdog Reset\n");
        g_eNextInterrupt = NEXT_SWPOR;
    }

    //
    // DEBUGGER.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_DEBUG)
    {
        am_util_stdio_printf("Debugger Initiated Reset\n");
        g_eNextInterrupt = NEXT_WATCHDOG;
    }

    //
    // SOFTWARE POI.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_POI)
    {
        am_util_stdio_printf(
                   "Software POI (power on reset internal state)\n");
        g_eNextInterrupt = NEXT_WATCHDOG;
    }

    //
    // SOFTWARE POR.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_SOFTWARE)
    {
        am_util_stdio_printf("Software POR Initiated Reset\n");
        g_eNextInterrupt = NEXT_SWPOI;
    }

    //
    // BROWNOUT DETECTOR.
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_BOD)
    {
        am_util_stdio_printf("Brownout Detector Initiated Reset\n");
        g_eNextInterrupt = NEXT_WATCHDOG;
    }

    //
    // EXTERNAL PIN
    //
    if (ui32ResetStatus & AM_HAL_RESET_STAT_EXTERNAL)
    {
        am_util_stdio_printf("External Reset Pin Initiated This Reset\n");
        g_eNextInterrupt = NEXT_SWPOI;
    }

    if ( g_eNextInterrupt == NEXT_WATCHDOG )
    {
        am_util_stdio_printf("  Next interrupt is WDT, so 5 interrupts should occur.\n");
    }
    else
    {
        am_util_stdio_printf("  Next interrupt is not WDT, so only 3 interrupts should occur.\n");
    }
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
    // Set g_eNextInterrupt to a watchdog interrupt.
    //
    g_eNextInterrupt = NEXT_WATCHDOG;

    //
    // Set system clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Stop the watch dog if we are coming in from a reset
    // other than a power cycle
    //
    am_hal_wdt_halt();

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the LED
    //
    am_devices_led_init(am_bsp_psLEDs);

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
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Reset State Tracking Example.\n");

    //
    // Decode and print the reset state that got us here.
    //
    reset_decode();

    //
    // Give user a little time to read the type of reset.
    //
    am_util_delay_ms(1000);

    //
    // Clear reset status register for next time we reset.
    //
    am_hal_reset_status_clear();

    //
    // LFRC has to be turned on for this example because the watchdog only
    // runs off of the LFRC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Configure the watchdog.
    //
    am_hal_wdt_init(&g_sWatchdogConfig);

    //
    // Enable the interrupt for the watchdog in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_WATCHDOG);
    am_hal_interrupt_master_enable();

    //
    // Enable the watchdog.
    //
    am_hal_wdt_start();

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // We are done printing. Disable debug printf messages on ITM.
        //
        am_bsp_debug_printf_disable();

        //
        // Disable interrupts.
        //
        am_hal_interrupt_master_disable();

        //
        // Turn OFF the indicator LED.
        //
        am_devices_led_off(am_bsp_psLEDs, 0);

        //
        // Go to sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        //
        // Turn ON the indicator LED.
        //
        am_devices_led_on(am_bsp_psLEDs, 0);

        //
        // An interrupt woke us up so now enable them and take it.
        //
        am_hal_interrupt_master_enable();
    }
}

