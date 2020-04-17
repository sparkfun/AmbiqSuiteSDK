//*****************************************************************************
//
//! @file flash_write_apollo2_main_block.c
//!
//! @brief Flash write examples for main block specifically for Apollo 2.
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
//
// Select an arbitrary page address in flash instance 1.
// We will arbitrarily pick 0x00082000 = 520*1024 = (512+8)*1024.
//
uint32_t g_ui32PrgmAddr = 520*1024;

//*****************************************************************************
//
// Main block erase function.
//
//*****************************************************************************
int
main_block_erase(void)
{
    int32_t i32ReturnCode;

    //
    // Erase the whole block of FLASH instance 1 (512KB to 1MB).
    //
    am_util_stdio_printf("  ... erasing all of flash instance %d.\n", AM_HAL_FLASH_ADDR2INST(g_ui32PrgmAddr) );
    i32ReturnCode = am_hal_flash_mass_erase(AM_HAL_FLASH_PROGRAM_KEY, 1);

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("ERROR: FLASH_MAIN_ERASE i32ReturnCode =  0x%x.\n",
                             i32ReturnCode);
    }

    return i32ReturnCode;

}

//*****************************************************************************
//
// Main block pattern write function.
//
//*****************************************************************************
int
main_block_pattern_write(void)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    uint32_t *pui32Src;
    uint32_t *pui32Dst;
    int32_t ix;

    //
    // Setup a pattern to write to the FLASH.
    //
    am_util_stdio_printf("  ... programming flash instance %d, page %d.\n",
                          AM_HAL_FLASH_ADDR2INST(g_ui32PrgmAddr),
                          AM_HAL_FLASH_ADDR2PAGE(g_ui32PrgmAddr) );

    pui32Src = g_ui32Source;
    for (ix = 0x100; ix < (0x100 +  AM_HAL_FLASH_INFO_SIZE); ix += 4)
    {
        *pui32Src++ = ix;
    }

    //
    // Program a "few" words in a page in the main block of instance 1.
    // We'll arbitrarily pick 0x00082000 = 520*1024 = (512+8)*1024.
    //
    pui32Dst = (uint32_t *) g_ui32PrgmAddr;
    i32ReturnCode = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                              g_ui32Source,
                                              pui32Dst,
                                              (AM_HAL_FLASH_INFO_SIZE >> 2));

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf("FLASH program page at 0x%08x "
                             "i32ReturnCode = 0x%x.\n",
                             g_ui32PrgmAddr,
                             i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check the page we just programmed.
    //
    am_util_stdio_printf("  ... verifying the page just programmed.\n");
    for ( ix = 0; ix <  (AM_HAL_FLASH_INFO_SIZE >> 2); ix++ )
    {
        if ( *(uint32_t*)(g_ui32PrgmAddr + (ix*4)) != g_ui32Source[ix] )
        {
            i32ErrorFlag++;
            am_util_stdio_printf("ERROR: flash address 0x%08x did not program properly:\n"
                                 "  Expected value = 0x%08x, programmed value = 0x%08x.\n",
                                 g_ui32PrgmAddr + (ix * 4),
                                 g_ui32Source[ix],
                                 *(uint32_t*)(g_ui32PrgmAddr + (ix * 4)) );
        }
    }

    return i32ErrorFlag;

}

//*****************************************************************************
//
// Main block page erasee function.
//
//*****************************************************************************
int
main_block_page_erase(void)
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    int32_t ix;

    //
    // Erase the page just programmed.
    //
    am_util_stdio_printf("  ... erasing the page just programmed @ 0x%08x.\n",
                             g_ui32PrgmAddr);
    i32ReturnCode = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                            AM_HAL_FLASH_ADDR2INST(g_ui32PrgmAddr),
                                            AM_HAL_FLASH_ADDR2PAGE(g_ui32PrgmAddr) );

    //
    // Check for an error from the HAL.
    //
    if (i32ReturnCode)
    {
        am_util_stdio_printf(" FLASH erase page at 0x%08x "
                             "i32ReturnCode =  0x%x.\n",
                             g_ui32PrgmAddr,
                             i32ReturnCode);
        i32ErrorFlag++;
    }

    //
    // Check that the entire page is erased.
    //
    am_util_stdio_printf("  ... verifying the page just erased.\n");
    for ( ix = 0; ix < ( AM_HAL_FLASH_INFO_SIZE >> 2); ix++ )
    {
        if ( *(uint32_t*)(g_ui32PrgmAddr + (ix * 4)) != 0xFFFFFFFF )
        {
            am_util_stdio_printf("ERROR: flash address 0x%08x did not ERASE properly:\n"
                                 "  Expected value = 0xFFFFFFFF, programmed value = 0x%08x.\n",
                                 g_ui32PrgmAddr + (ix*4),
                                 *(uint32_t*)(g_ui32PrgmAddr + (ix * 4)) );
        }
    }

    return i32ErrorFlag;
}
