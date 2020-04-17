//*****************************************************************************
//
//! @file uart_printf.c
//!
//! @brief Example that uses the UART interface for printf.
//!
//! This example walks through the ASCII table (starting at character 033('!')
//! and ending on 126('~')) and prints the character to the UART. This output
//! can be decoded by running AM Flash and configuring the console for UART at
//! 115200 Baud. This example works by configuring a timer and printing a new
//! character after ever interrupt and sleeps in between timer interrupts.
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Timer configurations.
//
//*****************************************************************************
am_hal_ctimer_config_t g_sTimer3 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_REPEAT |
     AM_HAL_CTIMER_INT_ENABLE    |
     AM_HAL_CTIMER_LFRC_512HZ),

    // No configuration for Timer0B.
    0,
};

//*****************************************************************************
//
// Init function for Timer A0.
//
//*****************************************************************************
void
timerA0_init(void)
{
    uint32_t ui32Period;

    //
    // Enable the LFRC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Set up timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer3);

    //
    // With LFRC running at 512 Hz, set up a print rate of 32 Hz.
    //  ui32Period = 8:  ~64 Hz print rate
    //  ui32Period = 16: ~32 Hz print rate
    //  ui32Period = 32: ~16 Hz print rate
    //
    ui32Period = 16;
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
                             (ui32Period >> 1));

    //
    // Clear the timer Interrupt
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear TimerA0 Interrupt (write to clear).
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}

//*****************************************************************************
//
// UART configuration settings.
//
//*****************************************************************************
am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate = 115200,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .bTwoStopBits = false,
    .ui32Parity   = AM_HAL_UART_PARITY_NONE,
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE,
};

//*****************************************************************************
//
// Initialize the UART
//
//*****************************************************************************
void
uart_init(int32_t i32Module)
{
    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    //
    // Power on the selected UART
    //
    am_hal_uart_pwrctrl_enable(i32Module);

    //
    // Start the UART interface, apply the desired configuration settings, and
    // enable the FIFOs.
    //
    am_hal_uart_clock_enable(i32Module);

    //
    // Disable the UART before configuring it.
    //
    am_hal_uart_disable(i32Module);

    //
    // Configure the UART.
    //
    am_hal_uart_config(i32Module, &g_sUartConfig);

    //
    // Enable the UART FIFO.
    //
    am_hal_uart_fifo_config(i32Module, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(i32Module);
}

//*****************************************************************************
//
// Enable the UART
//
//*****************************************************************************
void
uart_enable(int32_t i32Module)
{
    //
    // Enable the UART clock.
    //
    am_hal_uart_clock_enable(i32Module);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(i32Module);

    //
    // Enable the UART pins.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);
}

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
uart_disable(int32_t i32Module)
{
      //
      // Clear all interrupts before sleeping as having a pending UART interrupt
      // burns power.
      //
      am_hal_uart_int_clear(i32Module, 0xFFFFFFFF);

      //
      // Disable the UART.
      //
      am_hal_uart_disable(i32Module);

      //
      // Disable the UART pins.
      //
      am_bsp_pin_disable(COM_UART_TX);
      am_bsp_pin_disable(COM_UART_RX);

      //
      // Disable the UART clock.
      //
      am_hal_uart_clock_disable(i32Module);
}

//*****************************************************************************
//
// Transmit delay waits for busy bit to clear to allow
// for a transmission to fully complete before proceeding.
//
//*****************************************************************************
void
uart_transmit_delay(int32_t i32Module)
{
  //
  // Wait until busy bit clears to make sure UART fully transmitted last byte
  //
  while ( am_hal_uart_flags_get(i32Module) & AM_HAL_UART_FR_BUSY );
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t i;
    int32_t i32Module = AM_BSP_UART_PRINT_INST;

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
    // Initialize the printf interface for UART output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t)am_bsp_uart_string_print);

    //
    // Initialize and Enable the UART.
    //
    uart_init(i32Module);

    //
    // Print the banner.
    //
    am_util_stdio_printf("UART Printf Example\n");
    uart_transmit_delay(i32Module);
    am_util_stdio_printf("\n\tBeginning ASCII walk through:\n");
    uart_transmit_delay(i32Module);
    am_util_stdio_printf("\t");
    uart_transmit_delay(i32Module);

    //
    // Disable UART for now
    //
    uart_disable(i32Module);

    //
    // TimerA0 init.
    //
    timerA0_init();

    //
    // Enable the timer Interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();

    //
    // Enable the timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    //
    // Loop forever writing chars to the stimulus register.
    //
    while (1)
    {
        for (i = '!'; i <= '~'; i++)
        {
            //
            // Enable UART
            //
            uart_enable(i32Module);

            //
            // Walk through the ASCII table.
            //
            am_util_stdio_printf("%c", i);
            uart_transmit_delay(i32Module);

            //
            // Disable UART.
            //
            uart_disable(i32Module);

            //
            // Go to Deep Sleep to delay.
            //
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }

        //
        // Enable the UART, print a new line, and disable.
        //
        uart_enable(i32Module);
        am_util_stdio_printf("\n\t");
        uart_transmit_delay(i32Module);
        uart_disable(i32Module);
    }
}
