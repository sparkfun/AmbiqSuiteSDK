//*****************************************************************************
//
//! @file uart_printf.c
//!
//! @brief Example that uses the UART interface for printf.
//!
//! This example demonstrates the use of SW Buffer for UART.
//! It listens to the UART, and loops back the received data on the Tx
//!                                                         6
//! The UART is set at 115,200 BAUD, 8 bit, no parity.
//!
//! Finally, a banner, transmit, and receive status information is
//! printed to the ITM/SWO.
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
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define UART_BUFFER_SIZE    (128 * 2)
uint8_t g_pui8UartRxBuffer[UART_BUFFER_SIZE];
uint8_t g_pui8UartTxBuffer[UART_BUFFER_SIZE];
uint8_t g_ui8RxTimeoutFlag = 0;
uint8_t g_ui8BufferFullFlag = 0;

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
    // Configure the UART FIFO.
    //
    am_hal_uart_fifo_config(ui32Module, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Initialize the UART queues.
    //
    am_hal_uart_init_buffered(ui32Module, g_pui8UartRxBuffer, UART_BUFFER_SIZE,
                              g_pui8UartTxBuffer, UART_BUFFER_SIZE);
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
    // Enable the UART clock.
    //
    am_hal_uart_clock_enable(ui32Module);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(ui32Module);
    am_hal_uart_int_enable(ui32Module, AM_HAL_UART_INT_RX_TMOUT |
                                       AM_HAL_UART_INT_RX |
                                       AM_HAL_UART_INT_TXCMP);

    //
    // Enable the UART pins.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART + ui32Module);
}

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
uart_disable(uint32_t ui32Module)
{
      //
      // Clear all interrupts before sleeping as having a pending UART interrupt
      // burns power.
      //
      am_hal_uart_int_clear(ui32Module, 0xFFFFFFFF);

      //
      // Disable the UART.
      //
      am_hal_uart_disable(ui32Module);

      //
      // Disable the UART pins.
      //
      am_bsp_pin_disable(COM_UART_TX);
      am_bsp_pin_disable(COM_UART_RX);

      //
      // Disable the UART clock.
      //
      am_hal_uart_clock_disable(ui32Module);
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
// UART1 Interrupt Service Routine
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t status;
    uint32_t rxSize, txSize;

    status = am_hal_uart_int_status_get(0, false);

    am_hal_uart_int_clear(0, status);

    if (status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TX | AM_HAL_UART_INT_RX))
    {
        am_hal_uart_service_buffered_timeout_save(0, status);
    }

    if (status & (AM_HAL_UART_INT_RX_TMOUT))
    {
        g_ui8RxTimeoutFlag = 1;
        am_util_stdio_printf("\nTMOUT, ");
    }

    am_hal_uart_get_status_buffered(0, &rxSize, &txSize);

    if (status & (AM_HAL_UART_INT_RX))
    {
        am_util_stdio_printf(", RX ");
    }

    if (status & (AM_HAL_UART_INT_TXCMP))
    {
        am_util_stdio_printf(", TXCMP");
    }

    am_hal_uart_get_status_buffered(0, &rxSize, &txSize);

    if (rxSize >= UART_BUFFER_SIZE / 2)
    {
        g_ui8BufferFullFlag = 1;
        am_util_stdio_printf(", RXF");
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
    uint32_t ui32Module = AM_BSP_UART_PRINT_INST;
    char pcTempBuf[UART_BUFFER_SIZE + 1];
    uint32_t ui32Length;

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
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);
    am_bsp_pin_enable(ITM_SWO);
    am_bsp_debug_printf_enable();
    am_hal_itm_enable();

    //
    // Initialize and Enable the UART.
    //
    uart_init(ui32Module);
    uart_enable(ui32Module);

    am_hal_interrupt_master_enable();

    //
    // Print the banner.
    //
    am_util_stdio_printf("UART FIFO Example\n");
    am_util_stdio_printf("Note - the UART terminal operates at 115,200 BAUD, 8 bit, no parity.\n");
    uart_transmit_delay(ui32Module);
    am_util_stdio_printf("\n\tEcho back the UART received data.\n");
    uart_transmit_delay(ui32Module);
    am_util_stdio_printf("\t");
    uart_transmit_delay(ui32Module);

    //
    // Loop forever writing chars to the stimulus register.
    //
    while (1)
    {
        //
        // Need to retrieve and echo received date for loopback
        // if either all the data has been received, or if we are in danger
        // of overflowing the receive buffer
        //
        if (g_ui8RxTimeoutFlag || g_ui8BufferFullFlag)
        {

            g_ui8RxTimeoutFlag = 0;
            g_ui8BufferFullFlag = 0;

            memset(pcTempBuf, 0x00, sizeof(pcTempBuf));

            ui32Length = am_hal_uart_char_receive_buffered(ui32Module, pcTempBuf, UART_BUFFER_SIZE);

            if (ui32Length)
            {
                if ( pcTempBuf[0] == '\n' )
                {
                    am_util_stdio_printf("START_TX (char='\\n')");
                }
                else
                {
                    am_util_stdio_printf("START_TX (char='%c')", pcTempBuf[0]);
                }

                //
                // We have overallocated the pcTempBuf, to make sure ui32Length==UART_BUFFER_SIZE is ok
                //
                pcTempBuf[ui32Length] = 0;
                am_hal_uart_string_transmit_buffered(ui32Module, pcTempBuf);
            }
        }
    }
}
