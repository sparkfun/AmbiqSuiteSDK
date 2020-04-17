//*****************************************************************************
//
//! @file flash_write_apollo2_info_block.c
//!
//! @brief Flash write examples for info block specifically for Apollo 2.
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

extern uint32_t g_ui32Source[4096];

//*****************************************************************************
//
// Main block erase function.
//
//*****************************************************************************
int
info_block_erase(void)
{
    int32_t i32ReturnCode;

    //
    // Erase the whole block of FLASH instance 1 (512KB to 1MB).
    //
    am_util_stdio_printf("  ... erasing all of INFO block, flash instance 0.\n");
    i32ReturnCode = am_hal_flash_erase_info(AM_HAL_FLASH_PROGRAM_KEY, 0);

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR: FLASH_INFO_ERASE i32ReturnCode =  0x%x.\n",
                             i32ReturnCode);
    }

    return i32ReturnCode;

}

//*****************************************************************************
//
// INFO block pattern write function.
//
//*****************************************************************************
int
info_block_pattern_write(void)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    uint32_t *pui32Src;
    uint32_t *pui32Dst = (uint32_t *) AM_HAL_FLASH_INFO_ADDR;
    int32_t ix;

    //
    // Setup a pattern to write to the FLASH.
    //
    am_util_stdio_printf("  ... programming INFO block of flash instance 0.\n");

    pui32Src = g_ui32Source;
    for (ix = 0x100; ix < (0x100 +  AM_HAL_FLASH_INFO_SIZE); ix += 4)
    {
        *pui32Src++ = ix;
    }

    //
    // Program a "few" words in a page in the INFO block of instance 0.
    //
    i32ReturnCode = am_hal_flash_program_info(
                        AM_HAL_FLASH_PROGRAM_KEY,
                        0,                                  // we are only supporting INFO on instance 0.
                        g_ui32Source,
                        0,                                  // offset
                        (AM_HAL_FLASH_INFO_SIZE >> 2) );    // num words

    am_util_stdio_printf("\nINFO block size = 0x%x = %d\n",
                         AM_HAL_FLASH_INFO_SIZE, AM_HAL_FLASH_INFO_SIZE);

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("FLASH program page at 0x%08x "
                             "i32ReturnCode = 0x%x.\n",
                             pui32Dst,
                             i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check the page we just programmed.
    //
    am_util_stdio_printf("  ... verifying the page just programmed.\n");
    for ( ix = 0; ix <  (AM_HAL_FLASH_INFO_SIZE >> 2); ix++ )
    {
        if ( pui32Dst[ix] != g_ui32Source[ix] )
        {
            i32ErrorFlag++;
            am_util_stdio_printf("ERROR: flash address 0x%08x did not program properly:\n"
                                 "  Expected value = 0x%08x, programmed value = 0x%08x.\n",
                                 pui32Dst + (ix * 4),
                                 g_ui32Source[ix],
                                 pui32Dst[ix]);
        }
    }

    return i32ErrorFlag;

}

//*****************************************************************************
//
// INFO block write board name function.
//
//*****************************************************************************
int
info_block_write_board_name(char *pChar)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    char *pcharDst = (char *)( AM_HAL_FLASH_INFO_ADDR +
                               AM_HAL_FLASH_INFO_SIZE - 128 );

    //
    // First copy string to 32-bit aligned buffer.
    //
    am_util_string_strncpy((char *) g_ui32Source, pChar, 127);

    //
    // Now write the board name to the info block.
    //
    i32ReturnCode = am_hal_flash_program_info(
                        AM_HAL_FLASH_PROGRAM_KEY,
                        0,                                      // we are only supporting INFO on instance 0.
                        g_ui32Source,
                        ((AM_HAL_FLASH_INFO_SIZE - 128) / 4),   // offset
                        (128 / 4) );                            // num words

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR: am_hal_flash_program_info at offset 0x%08x "
                             "i32ReturnCode = 0x%x.\n",
                             ((AM_HAL_FLASH_INFO_SIZE - 128) / 4),  // offset
                             i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // That's a wrap so print it.
    //
    am_util_stdio_printf("info_block_write_board_name: <%s>\n", pcharDst);

    return i32ErrorFlag;
}

//*****************************************************************************
//
// INFO block write BLE address function.
//
//*****************************************************************************
#define MAX_DIE_X     (113)
#define MAX_DIE_Y     (110)
#define MAX_DIE_XY     (113*110)
#define MAX_WAFER_NUM (25)
// max value = X*Y*WAFER = 113 * 110 * 25 = 310750 --> 19 bits
// this leaves 5 bits of lotid info. So, we throw away most LOTID bits.
int
info_block_write_ble_address(uint64_t ui64BLEAddress)
{
    am_hal_mcuctrl_device_t  mcu_dev;
    uint64_t ui64WorkingBLEAddress;
    uint32_t ui32Work;
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    uint32_t *pui32Dst = (uint32_t *) ( AM_HAL_FLASH_INFO_ADDR +
                                        AM_HAL_FLASH_INFO_SIZE - 128 - 8);

    //
    // Fabricate a unique-ish 24 bit part id from the CHIPID registers.
    //
    am_hal_mcuctrl_device_info_get(&mcu_dev);
    ui32Work  = ((mcu_dev.ui32ChipID0 >>  0) & 0x3ff);              // X
    ui32Work += ((mcu_dev.ui32ChipID0 >> 10) & 0x3ff) * MAX_DIE_X;  // Y
    ui32Work += ((mcu_dev.ui32ChipID0 >> 20) & 0x01f) * MAX_DIE_XY; // WAFER
    ui32Work += ((mcu_dev.ui32ChipID0 >> 28) & 0x00f)               // LOTID
                 * MAX_DIE_XY * MAX_WAFER_NUM;
    ui64WorkingBLEAddress = (ui64BLEAddress & 0xFFFFFFFFFF000000ULL) |
                            ui32Work;

    //
    // Add the bluetooth address (lower 6 out of 8 bytes) to a 32-bit
    // aligned source array.
    //
    g_ui32Source[0] = (uint32_t) ui64WorkingBLEAddress ;
    g_ui32Source[1] = (uint32_t) (ui64WorkingBLEAddress >> 32);

    //
    // Now write the bluetooth addres to the info block.
    //
    i32ReturnCode = am_hal_flash_program_info(
                        AM_HAL_FLASH_PROGRAM_KEY,
                        0,                                          // we are only supporting INFO on instance 0.
                        g_ui32Source,
                        ((AM_HAL_FLASH_INFO_SIZE - 128 - 8) / 4),   // offset
                        (8 / 4) );                                  // num words

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR: am_hal_flash_program_info at offset 0x%08x "
                             "i32ReturnCode = 0x%x.\n",
                             ((AM_HAL_FLASH_INFO_SIZE - 128 - 8) / 4),  // offset
                             i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check to make sure we wrote it.
    //
    if ( (pui32Dst[0] != g_ui32Source[0]) || (pui32Dst[1] != g_ui32Source[1]) )
    {
        am_util_stdio_printf("ERROR: bluetooth addres at offset 0x%08x "
                             " not written correctly "
                             "0x%08x%08x != 0x%08x%08x \n",
                             ((AM_HAL_FLASH_INFO_SIZE - 128 - 8) / 4),  // offset
                             pui32Dst[1], pui32Dst[0],
                             g_ui32Source[1], g_ui32Source[0] );
        i32ErrorFlag++;
    }


    return i32ErrorFlag;
}

//*****************************************************************************
//
// INFO block testing erase disable functions.
//
//*****************************************************************************
int
info_block_disable_erase(void)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;

    am_util_stdio_printf("Check INFO erase disable state %d %s\n",
                 am_hal_flash_info_erase_disable_check(),
                 (am_hal_flash_info_erase_disable_check())  ?
                 "ERROR"                                    :
                 "AS EXPECTED" );

    //
    // Now disable INFO space erases
    //
    i32ReturnCode = am_hal_flash_info_erase_disable();

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR:am_hal_flash_info_erase_disable  "
                             "i32ReturnCode = 0x%x.\n", i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check the results.
    //
    am_util_stdio_printf("Check INFO erase disable state %d %s\n",
                 am_hal_flash_info_erase_disable_check(),
                 (!am_hal_flash_info_erase_disable_check()) ?
                 "ERROR"                                    :
                 "AS EXPECTED" );

    return i32ErrorFlag;
}

//*****************************************************************************
//
// INFO block testing program disable functions.
//
//*****************************************************************************
int
info_block_disable_program(uint32_t ui32Mask, uint32_t ui32ExpectMask)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;

    am_util_stdio_printf("Check INFO program disable state 0x%x %s\n",
                 am_hal_flash_info_program_disable_get(),
                 (am_hal_flash_info_program_disable_get() == ui32ExpectMask) ?
                 "ERROR"                                                     :
                 "AS EXPECTED" );

    //
    // Now disable INFO space erases
    //
    i32ReturnCode = am_hal_flash_info_program_disable(ui32Mask);

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR:am_hal_flash_info_program_disable  "
                             "i32ReturnCode = 0x%x.\n", i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check the results.
    //
    am_util_stdio_printf("Check INFO program disable state 0x%x %s\n",
                 am_hal_flash_info_program_disable_get(),
                 (am_hal_flash_info_program_disable_get() != ui32ExpectMask)    ?
                 "ERROR"                                                        :
                 "AS EXPECTED" );

    return i32ErrorFlag;
}

//*****************************************************************************
//
// INFO block testing flash wipe enable functions.
//
//*****************************************************************************
int
info_block_enable_flash_wipe(void)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;

    am_util_stdio_printf("Check INFO enable flash wipe state %d %s\n",
                 am_hal_flash_wipe_flash_enable_check(),
                 (am_hal_flash_wipe_flash_enable_check())   ?
                 "ERROR"                                    :
                 "AS EXPECTED" );

    //
    // Now enable flash wipe function in INFO space.
    //
    i32ReturnCode = am_hal_flash_wipe_flash_enable();

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR:am_hal_flash_info_erase_disable  "
                             "i32ReturnCode = 0x%x.\n", i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check the results.
    //
    am_util_stdio_printf("Check INFO erase disable state %d %s\n",
                 am_hal_flash_wipe_flash_enable_check(),
                 (!am_hal_flash_wipe_flash_enable_check())  ?
                 "ERROR"                                    :
                 "AS EXPECTED" );

    return i32ErrorFlag;
}
