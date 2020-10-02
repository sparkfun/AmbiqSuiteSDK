//*****************************************************************************
//
//! @file hello_world_uart.c
//!
//! @brief A simple "Hello World" example using the UART peripheral.
//!
//! This example prints a "Hello World" message with some device info
//! over UART at 115200 baud. To see the output of this program, run AMFlash,
//! and configure the console for UART. The example sleeps after it is done
//! printing.
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
    am_hal_uart_fifo_config(ui32Module, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(ui32Module);
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
uart_transmit_delay(uint32_t ui32Module)
{
  //
  // Wait until busy bit clears to make sure UART fully transmitted last byte
  //
  while ( am_hal_uart_flags_get(ui32Module) & AM_HAL_UART_FR_BUSY );
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    am_util_id_t sIdDevice;
    uint32_t ui32StrBuf;

    //
    // Select a UART module to use.
    //
    uint32_t ui32Module = AM_BSP_UART_PRINT_INST;

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
    // Configure and enable the UART.
    //
    uart_init(ui32Module);

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World!\n\n");
    uart_transmit_delay(ui32Module);

    //
    // Print the device info.
    //
    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    uart_transmit_delay(ui32Module);
    am_util_stdio_printf("Device type: %s\n",
         sIdDevice.pui8DeviceName);
    uart_transmit_delay(ui32Module);

    am_util_stdio_printf("Qualified: %s\n",
                         sIdDevice.sMcuCtrlDevice.ui32Qualified ?
                         "Yes" : "No");
    uart_transmit_delay(ui32Module);

    am_util_stdio_printf("Device Info:\n"
                         "\tPart number: 0x%08X\n"
                         "\tChip ID0:    0x%08X\n"
                         "\tChip ID1:    0x%08X\n"
                         "\tRevision:    0x%08X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );
    uart_transmit_delay(ui32Module);

    //
    // If not a multiple of 1024 bytes, append a plus sign to the KB.
    //
    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32FlashSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tFlash size:  %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32FlashSize,
                         sIdDevice.sMcuCtrlDevice.ui32FlashSize / 1024,
                         &ui32StrBuf);
    uart_transmit_delay(ui32Module);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32SRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tSRAM size:   %7d (%d KB%s)\n\n",
                         sIdDevice.sMcuCtrlDevice.ui32SRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32SRAMSize / 1024,
                         &ui32StrBuf);
    uart_transmit_delay(ui32Module);

    //
    // Print the compiler version.
    //
    am_util_stdio_printf("Compiler: %s\n", COMPILER_VERSION);
    uart_transmit_delay(ui32Module);

    //
    // We are done printing.
    // Disable the UART and interrupts
    //
    uart_disable(ui32Module);

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
