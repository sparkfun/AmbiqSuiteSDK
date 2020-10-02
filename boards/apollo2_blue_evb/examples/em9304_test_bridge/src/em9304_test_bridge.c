//*****************************************************************************
//
//! @file em9304_test_bridge.c
//!
//! @brief UART-to-SPI bridge for Bluetooth Direct Mode testing of EM9304.
//!
//! This project implements a UART to SPI bridge for Direct Mode testing of
//! the EM9304 BLE Controller.  The project uses UART0 and IOM5 in SPI mode.
//! HCI packets are provided over the UART which the Apollo2 transfers via
//! the SPI interface according to the EM9304 data sheet.  Responses from the
//! EM9304 are read from the SPI interface and relayed over the UART.  The
//! project uses the FIFOs and interrupts of the UART and IOM in order to
//! implement non-blocking processing of the next received packet from either
//! interface.
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices_em9304.h"
#include "am_devices_button.h"

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

#define MAX_UART_PACKET_SIZE    128
#define HCI_APOLLO_RESET_PIN            AM_BSP_GPIO_EM9304_RESET
//#define HCI_APOLLO_POWER_PIN            AM_BSP_GPIO_EM9304_POWER
//#define HCI_APOLLO_POWER_CFG            AM_BSP_GPIO_CFG_EM9304_POWER
#define HCI_CMD_TYPE                                 1       /*!< HCI command packet */
#define HCI_BRIDGE_UART                 0
//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint32_t g_pui32TxArray[MAX_UART_PACKET_SIZE / 4];
uint8_t *g_pui8TxArray = (uint8_t *)g_pui32TxArray;
uint8_t g_pui8RxArray[MAX_UART_PACKET_SIZE];
uint8_t g_pui8UARTTXBuffer[MAX_UART_PACKET_SIZE];
uint8_t g_pui8UARTRXBuffer[MAX_UART_PACKET_SIZE];

volatile bool   g_bEM9304HCIReady = false;
volatile bool   g_bRxTimeoutFlag = false;
volatile bool   g_bPTMToggleFlag = false;

//*****************************************************************************
//
// Set up a CTIMER at 32KHz
//
//*****************************************************************************
void
timer_init(void)
{
    am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
                                   AM_HAL_CTIMER_HFRC_3MHZ |
                                   AM_HAL_CTIMER_FN_REPEAT |
                                   AM_HAL_CTIMER_INT_ENABLE);

    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 49, 25);


    //
    // Start the timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    //
    // Enable the timer interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
}

//*****************************************************************************
//
// Initialize the EM9304 BLE Controller
//
//*****************************************************************************
void
em9304_init(void)
{
    //
    // Configurre the EM9303 pins.
    //
    am_hal_gpio_pin_config(HCI_APOLLO_RESET_PIN, AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pin_config(AM_BSP_GPIO_EM9304_PTM, AM_HAL_GPIO_OUTPUT);

    //
    // Assert RESET to the EM9304.
    //
    am_hal_gpio_out_bit_clear(HCI_APOLLO_RESET_PIN);

    //
    // Setup SPI interface for EM9304
    //
    am_devices_em9304_config_pins();
    am_devices_em9304_spi_init(g_sEm9304.ui32IOMModule, &g_sEm9304IOMConfigSPI);

    // enable interrupt.
    am_devices_em9304_enable_interrupt();
    //
    // Delay for 20ms to make sure the em device gets ready for commands.
    //
    am_util_delay_ms(5);

    //
    // Enable the IOM and GPIO interrupt handlers.
    //
    am_hal_gpio_out_bit_set(HCI_APOLLO_RESET_PIN);

    //am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);

    am_util_debug_printf("HciDrvRadioBoot complete\n");

    am_util_delay_ms(20);
}

//*****************************************************************************
//
// Initialize the UART
//
//*****************************************************************************
void
uart_init(uint32_t ui32Module)
{
    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    //
    // Power on the selected UART
    //
    am_hal_uart_pwrctrl_enable(ui32Module);

    //
    // Start the UART interface, apply the desired configuration settings, and
    // enable the FIFOs.
    //
    am_hal_uart_clock_enable(ui32Module);

    //
    // Disable the UART before configuring it.
    //
    am_hal_uart_disable(ui32Module);

    //
    // Configure the UART.
    //
    am_hal_uart_config(ui32Module, &g_sUartConfig);

    //
    // Enable the UART FIFO.
    //
    am_hal_uart_fifo_config(ui32Module, AM_HAL_UART_TX_FIFO_1_2 |
                                        AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(ui32Module);

#if AM_PART_APOLLO
    am_hal_uart_int_enable(ui32Module, AM_HAL_UART_INT_RX_TMOUT |
                                       AM_HAL_UART_INT_RX);
#else
    am_hal_uart_int_enable(ui32Module, AM_HAL_UART_INT_RX_TMOUT |
                                       AM_HAL_UART_INT_RX       |
                                       AM_HAL_UART_INT_TXCMP);
#endif

    //
    // Initialize the buffered UART.
    //
    am_hal_uart_init_buffered(ui32Module, g_pui8UARTRXBuffer,
                              MAX_UART_PACKET_SIZE,
                              g_pui8UARTTXBuffer, MAX_UART_PACKET_SIZE);

    //
    // Enable the UART interrupt handler.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART + ui32Module);
}

//*****************************************************************************
//
// Enable the UART
//
//*****************************************************************************
void
uart_enable(uint32_t ui32Module)
{
    //
    // Re-Initialize the buffered UART. This is to discard any trailing junk
    // from previous batch
    //
    am_hal_uart_init_buffered(ui32Module, g_pui8UARTRXBuffer, MAX_UART_PACKET_SIZE,
                            g_pui8UARTTXBuffer, MAX_UART_PACKET_SIZE);

    //
    // Enable the UART pins.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);
}


//*****************************************************************************
//
// Interrupt handler for the CTIMERs
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    uint32_t ui32Status;

    //
    // Check and clear any active CTIMER interrupts.
    //
    ui32Status = am_hal_ctimer_int_status_get(true);
    am_hal_ctimer_int_clear(ui32Status);

    //
    // If indicated, toggle the GPIO5 pin.
    //
    if (g_bPTMToggleFlag)
    {
      am_hal_gpio_out_bit_toggle(AM_BSP_GPIO_EM9304_PTM);
    }
}

//*****************************************************************************
//
// Interrupt handler for the GPIO module
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Check and clear the GPIO interrupt status
    //
    ui64Status = am_hal_gpio_int_status_get(true);
    am_hal_gpio_int_clear(ui64Status);

    //
    // Check to see if this was a wakeup event from the BLE radio.
    //
    if ( ui64Status & AM_HAL_GPIO_BIT(AM_BSP_GPIO_EM9304_INT) )
    {
        //
        // Indicate that HCI packet is ready from EM9304.
        //
        g_bEM9304HCIReady = true;
    }
}

//*****************************************************************************
//
// Interrupt handler for the UART
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the masked interrupt status from the UART.
    //
    ui32Status = am_hal_uart_int_status_get(HCI_BRIDGE_UART, true);

    //
    // Clear the UART interrupts.
    //
    am_hal_uart_int_clear(HCI_BRIDGE_UART, ui32Status);

    //
    // If there are TMOUT, RX or TX interrupts then service them.
    //
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TX | AM_HAL_UART_INT_RX))
    {
#ifdef AM_PART_APOLLO
      am_hal_uart_service_buffered(HCI_BRIDGE_UART, ui32Status);
#else
      am_hal_uart_service_buffered_timeout_save(HCI_BRIDGE_UART, ui32Status);
#endif
    }

    //
    // If there is a TMOUT interrupt, then indicate that to the main routine.
    //
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT))
    {
        g_bRxTimeoutFlag = true;
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
    uint32_t      ui32NumChars;

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
    // Enable the buttons for user interaction.
    //
    am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    // Initialize the CTIMERA0.
    timer_init();

    //
    // Initialize the EM9304 interface.
    //
    em9304_init();

    //
    // Initialize the UART, and set it as the default print interface.
    //
    uart_init(HCI_BRIDGE_UART);

    //
    // Initialize the ITM debug interface.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_uart_string_print);

    //
    // Enable BLE data ready interrupt
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);

    am_hal_interrupt_master_enable();

    //
    // Loop forever writing chars to the stimulus register.
    //
    while (1)
    {
        //
        // Check for any packets from the EM9304
        //
        if (g_bEM9304HCIReady)
        {
            //
            // Reset the GPIO flag
            //
            g_bEM9304HCIReady = false;

            //
            // Turn on the IOM for this operation.
            //
            //am_devices_em9304_spi_init(g_sEm9304.ui32IOMModule, &g_sEm9304IOMConfigSPI);
            am_devices_em9304_spi_awake(g_sEm9304.ui32IOMModule);

            ui32NumChars = am_devices_em9304_block_read(&g_sEm9304, g_pui32TxArray, 0);

            //
            // Disable IOM SPI pins and turn off the IOM after operation
            //
            am_devices_em9304_spi_sleep(g_sEm9304.ui32IOMModule);

            //
            // Transmit the received SPI packet over the UART.
            //
            for (uint32_t i = 0; i < ui32NumChars; i++)
            {
                am_hal_uart_char_transmit_buffered(AM_BSP_UART_BTLE_INST,
                                                   (char)g_pui8TxArray[i]);
            }
        }

        // Check for the completion of a UART receive operation.
        if (g_bRxTimeoutFlag)
        {
            //
            // Reset the RX timeout flag.
            //
            g_bRxTimeoutFlag = false;

            //
            // Check the UART RX Buffer for any received HCI packets.
            //
            ui32NumChars = am_hal_uart_char_receive_buffered(AM_BSP_UART_BTLE_INST,
                                                             (char *)g_pui8RxArray,
                                                             MAX_UART_PACKET_SIZE);
            if ( ui32NumChars > 0 )
            {
                //
                // Turn on the IOM for this operation.
                //
                am_devices_em9304_spi_awake(g_sEm9304.ui32IOMModule);

                //
                // Write the HCI packet to the EM9304.
                //
                am_devices_em9304_block_write(&g_sEm9304, g_pui8RxArray[0], &g_pui8RxArray[1], ui32NumChars - 1);

                //
                // Disable IOM SPI pins and turn off the IOM after operation
                //
                am_devices_em9304_spi_sleep(g_sEm9304.ui32IOMModule);

                g_bEM9304HCIReady = true;
            }
        }

        // Check if Button #0 has been pressed, indicating reset EM9304 into Production Test Mode (PTM).
        am_devices_button_tick(&am_bsp_psButtons[0]);
        if (am_devices_button_pressed(am_bsp_psButtons[0]))
        {
          // Start toggling the GPIO5 in the CTIMER ISR
          g_bPTMToggleFlag = true;

          // Assert RESET to the EM9304.
          am_hal_gpio_out_bit_clear(HCI_APOLLO_RESET_PIN);

          // Delay for 5ms to make sure the EM9304 is in PTM.
          am_util_delay_ms(5);

          // Deassert RESET to the EM9304.
          am_hal_gpio_out_bit_set(HCI_APOLLO_RESET_PIN);

          // Delay for 5ms to make sure the EM9304 is in PTM.
          am_util_delay_ms(5);

          // Stop toggling the GPIO5 in the CTIMER ISR
          g_bPTMToggleFlag = false;

        }

    } // while(1)
}
