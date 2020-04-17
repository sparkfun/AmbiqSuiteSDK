//*****************************************************************************
//
//! @file prime.c
//!
//! @brief Example that displays the timer count on the LEDs.
//!
//! This example consists of a non-optimized, brute-force routine for computing
//! the number of prime numbers between 1 and a given value, N. The routine
//! uses modulo operations to determine whether a value is prime or not. While
//! obviously not optimal, it is very useful for exercising the core.
//!
//! For this example, N is 100000, for which the answer is 9592.
//!
//! For Apollo3 at 48MHz, the time to compute the answer for Keil and IAR:
//!     IAR v8.11.1:        1:43.
//!     Keil ARMCC 4060528: 1:55.
//!
//! Apollo2 at 48MHz takes less than 2 1/4 minutes to determine that answer
//! when compiled with IAR v8.11.
//!
//! The goal of this example is to measure current consumption while the core
//! is working to compute the answer. Power and energy can then be derived
//! knowing the current and run time.
//!
//! The example prints an initial banner to the UART port.  After each prime
//! loop, it enables the UART long enough to print the answer, disables the
//! UART and starts the computation again.
//!
//! Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
//! Please note that text end-of-line is a newline (LF) character only.
//! Therefore, the UART terminal must be set to simulate a CR/LF.
//!
//! Note: For minimum power, disable the printing by setting PRINT_UART to 0.
//!
//! The prime_number() routine is open source and is used here under the
//! GNU LESSER GENERAL PUBLIC LICENSE Version 3, 29 June 2007.  Details,
//! documentation, and the full license for this routine can be found in
//! the third_party/prime_mpi/ directory of the SDK.
//!
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
// Prototype for third-party algorithm.
//
//*****************************************************************************
// n: number of primes, id=process number (), p=number of processes (1)
int prime_number ( int n, int id, int p );

//*****************************************************************************
//
// For minimal power usage while the PRIME example is running, disable the
// UART by setting this define to 0.
//
//*****************************************************************************
#define PRINT_UART    1

//*****************************************************************************
//
// Number of primes to count.
//
//*****************************************************************************
//
// Set target to 100000 (for which the correct answer is 9592).
//
#define NUM_OF_PRIMES_IN 100000
#define EXP_PRIMES 9592

//*****************************************************************************
//
// Insert compiler version at compile time.
//
//*****************************************************************************
#define STRINGIZE_VAL(n)                    STRINGIZE_VAL2(n)
#define STRINGIZE_VAL2(n)                   #n

#ifdef __GNUC__
#define COMPILER_VERSION                    ("GCC " __VERSION__)
#elif defined(__ARMCC_VERSION)
#define COMPILER_VERSION                    ("ARMCC " STRINGIZE_VAL(__ARMCC_VERSION))
#elif defined(__KEIL__)
#define COMPILER_VERSION                    "KEIL_CARM " STRINGIZE_VAL(__CA__)
#elif defined(__IAR_SYSTEMS_ICC__)
#define COMPILER_VERSION                    __VERSION__
#else
#define COMPILER_VERSION                    "Compiler unknown"
#endif

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
uart_init(uint32_t ui32UartModule)
{
    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    //
    // Power on the selected UART
    //
    am_hal_uart_pwrctrl_enable(ui32UartModule);

    //
    // Start the UART interface, apply the desired configuration settings, and
    // enable the FIFOs.
    //
    am_hal_uart_clock_enable(ui32UartModule);

    //
    // Disable the UART before configuring it.
    //
    am_hal_uart_disable(ui32UartModule);

    //
    // Configure the UART.
    //
    am_hal_uart_config(ui32UartModule, &g_sUartConfig);

    //
    // Enable the UART FIFO.
    //
    am_hal_uart_fifo_config(ui32UartModule, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(ui32UartModule);
}

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
uart_disable(uint32_t ui32UartModule)
{
    //
    // Before disabling the UART, wait a little time to be sure all
    // printing has completed.
    //
    am_util_delay_ms(10);

    //
    // Disable and power down the UART.
    //
    am_hal_uart_disable(ui32UartModule);
    am_hal_uart_pwrctrl_disable(ui32UartModule);

    //
    // Turn off UART clock.
    // Note - this is automatically handled in hardware on Apollo2.
    //
    am_hal_uart_clock_disable(ui32UartModule);

    //
    // Disable the UART pins.
    //
    am_bsp_pin_disable(COM_UART_TX);
    am_bsp_pin_disable(COM_UART_RX);
}


//*****************************************************************************
//
// Minimize power
//
//*****************************************************************************
void
set_for_min_power(void)
{
    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Turn OFF Flash1.
    //
    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEMEN_FLASH512K);

    //
    // Turn off SRAMs above 8K.
    // Note - assumes a 4KB stack (the usual example stack size).
    //
    am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEMEN_SRAM8K);

    //
    // Let the XTAL turn off.
    //
    AM_BFW(CLKGEN, OCTRL, OSEL, 1);

    //
    // Turn off the voltage comparator.
    //
    AM_REG(VCOMP, PWDKEY) = AM_REG_VCOMP_PWDKEY_KEYVAL;

#if AM_PART_APOLLO2
    //
    // Powerdown the BOD and PDR logic.
    //
    AM_REG(MCUCTRL, BODPORCTRL) = 0x2;  // 3=disable both 2=disable BOD 1=disable PDR

    //
    // Turn off all peripheral power domains.
    //
    AM_REG(PWRCTRL, DEVICEEN) = 0;
#endif // AM_PART_APOLLO2
}

//*****************************************************************************
//
// Main Function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Result;
    uint32_t ui32UartModule = AM_BSP_UART_PRINT_INST;

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

#if (PRINT_UART == 1)
    //
    // Initialize the printf interface for UART output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t)am_bsp_uart_string_print);

    //
    // Initialize the UART print interface
    //
    uart_init(ui32UartModule);

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Ambiq Micro 'prime' example.\n\n");

    //
    // Brief description
    //
    am_util_stdio_printf("Used for measuring power while computing the number of prime numbers in a range.\n");

    //
    // Print the compiler version.
    //
    am_util_stdio_printf("Compiler: %s\n", COMPILER_VERSION);

    //
    // To minimize power during the run, disable the UART.
    //
    uart_disable(ui32UartModule);
#endif // PRINT_UART

    //
    // Call the prime function
    //
    while(1)
    {
        //
        // Set MCU for minimal power
        //
        set_for_min_power();

        //
        // Determine the number of primes for the given value.
        //
        ui32Result = prime_number(NUM_OF_PRIMES_IN, 0, 1);

#if (PRINT_UART == 1)
        //
        // Print the result
        //
        uart_init(ui32UartModule);

        if ( ui32Result == EXP_PRIMES )
        {
            am_util_stdio_printf("Pass: number of primes for %d is %d.\n", NUM_OF_PRIMES_IN, ui32Result);
        }
        else
        {
            am_util_stdio_printf("ERROR: Invalid result. Expected %d, got %d.\n", NUM_OF_PRIMES_IN, ui32Result);
        }

        uart_disable(ui32UartModule);
#endif // PRINT_UART
    }
}
