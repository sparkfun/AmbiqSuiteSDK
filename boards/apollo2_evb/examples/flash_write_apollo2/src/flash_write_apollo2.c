//*****************************************************************************
//
//! @file flash_write_apollo2.c
//!
//! @brief Flash write example specifically for Apollo 2.
//!
//! This example shows how to modify the internal Flash using HAL flash helper
//! functions.
//!
//! This example works on instance 1 of the Flash, i.e. the portion of the
//! Flash above 512KB. It is intended to execute from instance 0.
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

#include "flash_write_apollo2_main_block.h"
#include "flash_write_apollo2_info_block.h"

uint32_t g_ui32Source[4096]; // 2 8KB pages worth.

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    int32_t i32ErrorFlag = 0;

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
    am_util_stdio_printf("Flash Write Example\n");

    //
    // Erase the whole block of FLASH instance 1 (512KB to 1MB).
    //
    if ( main_block_erase() )
    {
        i32ErrorFlag++;
    }

    //
    // Program a few words in a page in the main block of instance 1.
    // We'll arbitrarily pick 0x00041000 = 260*1024 = (256+4)*1024.
    //
    if ( main_block_pattern_write() )
    {
        i32ErrorFlag++;
    }

    //
    // Erase the page just programmed.
    //
    if ( main_block_page_erase() )
    {
        i32ErrorFlag++;
    }

    //
    // Erase the INFO block.
    //
    if ( info_block_erase() )
    {
        i32ErrorFlag++;
    }

    //
    // Program a few words in the INFO block.
    //
    if ( info_block_pattern_write() )
    {
        i32ErrorFlag++;
    }

    //
    // Erase the INFO block again before the next two steps.
    //
    if ( info_block_erase() )
    {
        i32ErrorFlag++;
    }

    //
    // Write a board name into the INFO block at 128 bytes before the end.
    //
    if ( info_block_write_board_name("CUSTOMER: MY BOARD") )
    {
        i32ErrorFlag++;
    }

    //
    // Write 6 byte BLE Address into the INFO block at 136 bytes before the end.
    //
    if ( info_block_write_ble_address(0x0000123456ABCDEF) )
    {
        i32ErrorFlag++;
    }

    //
    // Disable Flash INFO space erase operations.
    //
    if ( info_block_disable_erase() )
    {
        i32ErrorFlag++;
    }

    //
    // Disable Flash INFO space program operations by quadrant.
    //
    if ( info_block_disable_program(0xA, 0xA) )
    {
        i32ErrorFlag++;
    }

    //
    // Enable flash wipe function in Flash INFO space.
    //
    if ( info_block_enable_flash_wipe() )
    {
        i32ErrorFlag++;
    }

    //
    // Report success or any failures.
    //
    if (i32ErrorFlag)
    {
        am_util_stdio_printf("ERROR: FLASH Write example failure %d\n",
                             i32ErrorFlag);
    }
    else
    {
        am_util_stdio_printf("FLASH Write example successful \n");
    }

    //
    // Verify that ITM has finished printing.
    //
    am_hal_itm_not_busy();


    //
    // Wait at least 20 seconds.
    //
    am_hal_flash_delay(20 * (AM_HAL_CLKGEN_FREQ_MAX_HZ / 3));

    //
    // Unconditionally recover a part that could be bricked.
    // We will never return from this call.
    //
    am_hal_flash_recovery(AM_HAL_FLASH_RECOVERY_KEY);


    //
    // Provide return code back to the system.
    //
    return i32ErrorFlag;
}
