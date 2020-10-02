//*****************************************************************************
//
//! @file deepsleep_wake.c
//!
//! @brief Example that goes to deepsleep and wakes from either the RTC or GPIO.
//!
//! This example configures the device to go into a deep sleep mode. Once in
//! deep sleep the device has the ability to wake from button 0 or
//! the RTC configured to interrupt every second. If the MCU woke from a button
//! press, it will toggle LED0. If the MCU woke from the RTC, it will toggle
//! LED1.
//!
//! The example begins by printing out a banner annoucement message through
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

// RTC time structure.
am_hal_rtc_time_t g_sTime;

#define XT              1
#define LFRC            2

#define RTC_CLK_SRC     XT

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
// GPIO ISR
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    //
    // Delay for debounce.
    //
    am_util_delay_ms(200);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));

    //
    // Toggle LED 0.
    //
    am_devices_led_toggle(am_bsp_psLEDs, 0);
}

//*****************************************************************************
//
// RTC ISR
//
//*****************************************************************************
void
am_clkgen_isr(void)
{
    //
    // Clear the RTC alarm interrupt.
    //
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    //
    // Toggle LED 1.
    //
    am_devices_led_toggle(am_bsp_psLEDs, 1);
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

#if RTC_CLK_SRC == LFRC
    //
    // Enable the LFRC for the RTC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Select LFRC for RTC clock source.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);
#endif

#if RTC_CLK_SRC == XT
    //
    // Enable the XT for the RTC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_XT);

    //
    // Select XT for RTC clock source
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
#endif

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();

    //
    // Initialize the printf interface for UART output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t)am_bsp_uart_string_print);

    //
    // Initialize the UART
    //
    uart_init(AM_BSP_UART_PRINT_INST);

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Deepsleep Wake Example\n");

    //
    // To minimize power during the run, disable the UART.
    //
    uart_disable(AM_BSP_UART_PRINT_INST);

#ifdef AM_PART_APOLLO
    //
    // Power down all but the first SRAM banks.
    //
    am_hal_mcuctrl_sram_power_set(AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7,
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7);
#endif // AM_PART_APOLLO

#ifdef AM_PART_APOLLO2

    //
    // Turn OFF Flash1
    //
    AM_BFW(PWRCTRL, MEMEN, FLASH1, 0);
    while (AM_BFR(PWRCTRL, PWRONSTATUS, PD_FLAM1) != 0) {}

    //
    // Power down SRAM
    //
    AM_BFWe(PWRCTRL, SRAMPWDINSLEEP, SRAMSLEEPPOWERDOWN, ALLBUTLOWER8K);
#endif // AM_PART_APOLLO2

    //
    // Configure the LEDs GPIO and button pins.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
    am_hal_gpio_pin_config(AM_BSP_GPIO_BUTTON0, AM_HAL_GPIO_INPUT);

    //
    // Turn the LEDs off.
    //
    am_devices_led_off(am_bsp_psLEDs, 0);
    am_devices_led_off(am_bsp_psLEDs, 1);

    //
    // Configure the GPIO/button interrupt polarity.
    //
    am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_BUTTON0, AM_HAL_GPIO_RISING);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));

    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));

    //
    // Set the alarm repeat interval to be every second.
    //
    am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_SEC);

    //
    // Clear the RTC alarm interrupt.
    //
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    //
    // Enable the RTC alarm interrupt.
    //
    am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);

    //
    // Enable GPIO interrupts to the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CLKGEN);

    //
    // Enable interrupts to the core.
    //
    am_hal_interrupt_master_enable();

    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
