//*****************************************************************************
//
//! @file pwm_gen.c
//!
//! @brief Breathing LED example.
//!
//! This example shows one way to vary the brightness of an LED using timers in
//! PWM mode.
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
// LED brightness profile
//
//*****************************************************************************
volatile uint32_t g_ui32Index = 0;
const uint32_t g_pui32Brightness[64] =
{
    1, 1, 1, 2, 3, 4, 6, 8,
    10, 12, 14, 17, 20, 23, 25, 28,
    31, 35, 38, 40, 43, 46, 49, 51,
    53, 55, 57, 59, 60, 61, 62, 62,
    63, 62, 62, 61, 60, 59, 57, 55,
    53, 51, 49, 46, 43, 40, 38, 35,
    32, 28, 25, 23, 20, 17, 14, 12,
    10, 8, 6, 4, 3, 2, 1, 1
};

//*****************************************************************************
//
// Timer Interrupt Serive Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear the interrupt that got us here.
    //
    am_hal_ctimer_int_clear(AM_BSP_PWM_LED_TIMER_INT);

    //
    // Now set new PWM half-period for the LED.
    //
    am_hal_ctimer_period_set(AM_BSP_PWM_LED_TIMER, AM_BSP_PWM_LED_TIMER_SEG,
                             64, g_pui32Brightness[g_ui32Index]);

    //
    // Set up the LED duty cycle for the next pulse.
    //
    g_ui32Index = (g_ui32Index + 1) % 64;
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
    // Among other things, am_bsp_low_power_init() stops the XT oscillator,
    // which is needed for this example.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_XT);

    //
    // Configure the pins for this example.
    //
    am_bsp_pin_enable(PWM_LED);

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
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("PWM Example\n\n");
    am_util_delay_ms(10);

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();

    //
    // Configure a timer to drive the LED.
    //
    am_hal_ctimer_config_single(AM_BSP_PWM_LED_TIMER, AM_BSP_PWM_LED_TIMER_SEG,
                                (AM_HAL_CTIMER_FN_PWM_REPEAT |
                                 AM_HAL_CTIMER_XT_2_048KHZ |
                                 AM_HAL_CTIMER_INT_ENABLE |
                                 AM_HAL_CTIMER_PIN_ENABLE));

    //
    // Set up initial timer period.
    //
    am_hal_ctimer_period_set(AM_BSP_PWM_LED_TIMER, AM_BSP_PWM_LED_TIMER_SEG,
                             64, 32);

    //
    // Enable interrupts for the Timer we are using on this board.
    //
    am_hal_ctimer_int_enable(AM_BSP_PWM_LED_TIMER_INT);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();

    //
    // Start the timer.
    //
    am_hal_ctimer_start(AM_BSP_PWM_LED_TIMER, AM_BSP_PWM_LED_TIMER_SEG);

    //
    // Sleep forever while waiting for an interrupt.
    //
    while (1)
    {

        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
