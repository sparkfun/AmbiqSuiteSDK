//*****************************************************************************
//
//  am_hal_flash.c
//! @file
//!
//! @brief Functions for performing Flash operations.
//!
//! @addtogroup flash1 Flash
//! @ingroup apollo1hal
//! @{
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

//
// Look-up table
//
const g_am_hal_flash_t g_am_hal_flash =
{
    // am_hal_flash_mass_erase
    ((int  (*)(uint32_t, uint32_t)) 0x0800004d),
    // am_hal_flash_page_erase
    ((int  (*)(uint32_t, uint32_t, uint32_t))  0x08000051),
    // am_hal_flash_program_main
    ((int  (*)(uint32_t, const uint32_t *, uint32_t *, uint32_t))  0x08000055),
    // am_hal_flash_program_otp
    ((int  (*)(uint32_t, uint32_t, const uint32_t *, uint32_t, uint32_t)) 0x08000059),
    // am_hal_flash_program_main_sram
    ((void (*)(void))  0x0800005d),
    // am_hal_flash_program_otp_sram
    ((void (*)(void))  0x08000061),
    // am_hal_flash_erase_main_pages_sram
    ((void (*)(void))  0x08000065),
    // am_hal_flash_mass_erase_sram
    ((void (*)(void))  0x08000069)
};

//
// Set up a very small function that will be guaranteed to be located in SRAM
//  which can be used to to retrieve data from OTP.
// Make sure the function is word-aligned.
//
uint32_t SRAM_load_ui32[8 / 4] =
{
    0xBF006800,         // 6800   ldr    r0,[r0,#0]
                        // BF00   nop
    0xBF004770          // 4770   bx lr
                        // BF00   nop
};

//*****************************************************************************
//
//! @brief Implement an iterative spin loop.
//!
//! @param ui32Iterations - Number of iterations to delay.
//!
//! Use this function to implement a CPU busy waiting spin.  For Apollo, this
//! delay can be used for timing purposes since for Apollo, each iteration will
//! take 3 cycles.
//!
//! @return None.
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__asm void
am_hal_flash_delay(uint32_t ui32Iterations)
{
    SUBS    R0, #1
    BNE     am_hal_flash_delay
    BX      LR
}
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
void __attribute__((naked))
am_hal_flash_delay(uint32_t ui32Iterations)
{
    __asm("    subs    r0, #1\n"
          "    bne     am_hal_flash_delay\n"
          "    bx      lr");
}
#elif defined(__GNUC_STDC_INLINE__)
void __attribute__((naked))
am_hal_flash_delay(uint32_t ui32Iterations)
{
    __asm("    subs    r0, #1\n"
          "    bne     am_hal_flash_delay\n"
          "    bx      lr");
}
#elif defined(__IAR_SYSTEMS_ICC__)
void
am_hal_flash_delay(uint32_t ui32Iterations)
{
    asm("SUBS    R0, #1");
    asm("BNE.N   am_hal_flash_delay");
    asm("BX      LR");
}
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles while also waiting for a
//! status change.
//!
//! @param ui32usMaxDelay - Maximum number of ~1uS delay loops.
//! @param ui32Address    - Address of the register for the status change.
//! @param ui32Mask       - Mask for the status change.
//! @param ui32Value      - Target value for the status change.
//!
//! This function will delay for approximately the given number of microseconds
//! while checking for a status change, exiting when either the given time has
//! expired or the status change is detected.
//!
//! @returns 0 = timeout.
//!          1 = status change detected.
//
//*****************************************************************************
uint32_t
am_hal_flash_delay_status_change(uint32_t ui32usMaxDelay, uint32_t ui32Address,
                                 uint32_t ui32Mask, uint32_t ui32Value)
{
    while ( ui32usMaxDelay-- )
    {
        //
        // Check the status
        //
        if ( ( AM_REGVAL(ui32Address) & ui32Mask ) == ui32Value )
        {
            return 1;
        }

        //
        // Call the BOOTROM cycle function to delay for about 1 microsecond.
        //
        am_hal_flash_delay( FLASH_CYCLES_US(1) );
    }

    return 0;
} // am_hal_flash_delay_status_change()

//*****************************************************************************
//
//! @brief This function performs a mass erase on a flash block.
//!
//! @param ui32Value - The flash program key.
//! @param ui32FlashBlk - The flash block to erase.
//!
//! This function will erase the desired block of flash.
//!
//! @note Each flash block contains a maximum of 256kB.
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
int
am_hal_flash_mass_erase(uint32_t ui32Value, uint32_t ui32FlashBlk)
{
    return g_am_hal_flash.am_hal_flash_mass_erase(ui32Value, ui32FlashBlk);
}

//*****************************************************************************
//
//! @brief This function performs a page erase on a flash block.
//!
//! @param ui32Value - The flash program key.
//! @param ui32FlashBlk - The flash block to reference the page number with.
//! @param ui32PageNum - The flash page offset into the selected block.
//!
//! This function will erase the desired flash page in the desired block of
//! flash.
//!
//! @note For Apollo, each flash page is 2KB (or AM_HAL_FLASH_PAGE_SIZE).
//! Each flash block (instance) contains a maximum of 128 pages
//! (or AM_HAL_FLASH_BLOCK_PAGES).
//!
//! @note When given an absolute flash address, a couple of helpful macros can
//! be utilized when calling this function.
//! For example:
//!     am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
//!                             AM_HAL_FLASH_ADDR2BLOCK(ui32Addr),
//!                             AM_HAL_FLASH_ADDR2PAGE(ui32Addr) );
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
int
am_hal_flash_page_erase(uint32_t ui32Value, uint32_t ui32FlashBlk,
                        uint32_t ui32PageNum)
{
    return g_am_hal_flash.am_hal_flash_page_erase(ui32Value, ui32FlashBlk,
                                                  ui32PageNum);
}

//*****************************************************************************
//
//! @brief This programs up to N bytes of the Main array on one flash block.
//!
//! @param ui32Value - The Program key.
//! @param pui32Src - Pointer to word aligned array of data to program into
//! the flash block.
//! @param pui32Dst - Pointer to word aligned location to to begin programming
//! the flash block.
//! @param ui32NumWords - The Number of words to program.
//!
//! This function will program multiple words in the OTP block on flash
//! block (instance) 0.
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
int
am_hal_flash_program_main(uint32_t ui32Value, const uint32_t *pui32Src,
                          uint32_t *pui32Dst, uint32_t ui32NumWords)
{
    return g_am_hal_flash.am_hal_flash_program_main(ui32Value, pui32Src,
                                                    pui32Dst, ui32NumWords);
}

//*****************************************************************************
//
//! @brief This function programs multiple words in the OTP.
//!
//! @param ui32Value - The OTP key.
//! @param ui32FlashBlk - The flash block where OTP lives (forced to block 0)
//! @param *pui32Src - Pointer to word aligned array of data to program into
//! the OTP block.
//! @param ui32Offset - Word offset into OTP (offset of 0 is the first word).
//! @param ui32NumWords - The Number of words to program.
//!
//! This function will program multiple words in the OTP block on flash
//! block (instance) 0.
//!
//! @note Only the upper half of the OTP block can be written
//!       so 1 < ui32NumWords < 257.
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
int
am_hal_flash_program_otp(uint32_t ui32Value,       uint32_t ui32FlashBlk,
                         const uint32_t *pui32Src, uint32_t ui32Offset,
                         uint32_t ui32NumWords)
{
    return g_am_hal_flash.am_hal_flash_program_otp(ui32Value, 0, pui32Src,
                                                   ui32Offset, ui32NumWords);
}

//*****************************************************************************
//
//! @brief This function programs multiple words in the OTP using SRAM as args.
//!
//! This function will program multiple words in the OTP block using SRAM as
//! its arguments. This is helpful for tools/manufacturing.
//!
//! The SRAM addresses of interest are:
//!
//!     0x10000000   Offset in to OTP block, 0 <= Offset < 256
//!     0x10000004     Number of 32-bit words to program
//!     0x10000008     OTP key
//!     0x1000000C     Debugger sets this to -1 and all return codes are >= 0
//!     0x10000010     First 32-bit word of data buffer to be programmed
//!
//! @note This routine spins when am_hal_flash_program_otp() returns and waits
//! for the debugger surrogate in the parallel programmer.
//!
//! @return never returns, spins here waiting for debugger or debugger surrogate
//! on the parallel programmer.
//
//*****************************************************************************
void
am_hal_flash_program_otp_sram(void)
{
    g_am_hal_flash.am_hal_flash_program_otp_sram();
}

//*****************************************************************************
//
//! @brief This function erases pages in the main array using SRAM as args.
//!
//! This function will erase multiple pages in the flash main array using SRAM
//! as its arguments.  This is helpful for tools/manufacturing.
//!
//! The SRAM addresses of interest are:
//!
//!     0x10000000   Flash block/instance number
//!     0x10000004   Number of pages to erase  (must be between 1 and 128
//!     inclusive)
//!     0x10000008   PROGRAM key
//!     0x1000000C   Debugger sets this to -1 and all return codes are >= 0
//!     0x10000010   Page number of the first flash page to erase. NOTE: these
//!                   *HAVE* to be sequential (range 0 <= PageNumber <= 127)
//!
//! @note This routine spins when am_hal_flash_page_erase() returns and waits
//! for the debugger surrogate in the parallel programmer. Before spinning it
//! unconditional executes a break point instruction.
//!
//! @return never returns, spins here waiting for debugger or debugger surrogate
//! on the parallel programmer.
//
//*****************************************************************************
void
am_hal_flash_erase_main_pages_sram(void)
{
    g_am_hal_flash.am_hal_flash_erase_main_pages_sram();
}

//*****************************************************************************
//
//! @brief This function mass erases a flash block with SRAM as args.
//!
//! This function will perform a mass erase on a flash block using SRAM as its
//! arguments. This is helpful for tools/manufacturing.
//!
//! The SRAM addresses of interest are:
//!
//!     0x10000000   Pointer in to flash block.
//!     0x10000004   PROGRAM key
//!     0x10000008   Return code (Debugger sets this to -1 and all return
//!                               codes are >= 0)
//!
//! @note This routine spins when am_hal_flash_mass_erase() returns and waits
//! for the debugger surrogate in the parallel programmer.
//!
//! @return never returns, spins here waiting for debugger or debugger surrogate
//! on the parallel programmer.
//
//*****************************************************************************
void
am_hal_flash_mass_erase_sram(void)
{
    g_am_hal_flash.am_hal_flash_mass_erase_sram();
}

//*****************************************************************************
//
//! @brief This function programs the main array using SRAM as args.
//!
//! This function will program the main array using SRAM as it arguments.  This
//! is helpful for tools/manufacturing.
//!
//! The SRAM addresses of interest are:
//!
//!     0x10000000   Pointer in to flash
//!     0x10000004   Number of 32-bit words to program
//!     0x10000008   PROGRAM key.
//!     0x1000000C   Debugger sets this to -1 and all return codes are >= 0
//!     0x10000010   First 32-bit word of data buffer to be programmed
//!
//! @note This routine spins when am_hal_flash_program_main() returns and waits
//! for the debugger surrogate in the parallel programmer. Before spinning it
//! unconditionally executes a break point instruction.
//!
//! @return never returns, spins here waiting for debugger or debugger surrogate
//! on the parallel programmer.
//
//*****************************************************************************
void
am_hal_flash_program_main_sram(void)
{
    g_am_hal_flash.am_hal_flash_program_main_sram();
}

//*****************************************************************************
//
//! @brief Return ui32 value obtained from anywhere in D Code or System Bus
//!
//! @param ui32Address - return the value corresponding to this location in OTP
//!
//! Use this function to read a value from various peripheral locations
//! that must be read from code running in SRAM.
//!
//! This function is not required to reside in SRAM as it calls a small
//! function that does the actual read which is guaranteed to be in SRAM.
//!
//! @return the value found
//
//*****************************************************************************
uint32_t
am_hal_flash_load_ui32(uint32_t ui32Address)
{
    //
    // Call the simple routine that has been set up in SRAM as an array.
    // First set up a function pointer to the array, being sure to set the
    //  .T bit (Thumb bit, bit0) in the branch address, then use that
    //  function ptr to call the SRAM function.
    //
    uint32_t SRAMCode = (uint32_t)SRAM_load_ui32 | 0x1;
    uint32_t (*pFunc)(uint32_t) = (uint32_t (*)(uint32_t))SRAMCode;
    return (*pFunc)(ui32Address);
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
