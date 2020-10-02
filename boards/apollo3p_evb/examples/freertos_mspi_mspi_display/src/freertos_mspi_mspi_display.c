//*****************************************************************************
//
//! @file freertos_mspi_iom_display.c
//!
//! @brief Example demonstrating frame buffer compositing and display rendering
//!
//! This example demonstrates frame buffer compositing and display rendering
//! using the hardware assisted MSPI to IOM transfer under FreeRTOS.
//! To demonstrate full speed frame composition and rendering, a ping-pong
//! Frame buffer is used in the PSRAM, and composition is done in parallel,
//! while rendering is ongoing from alternate frame buffer
//!
//! At initialization, both the Display and PSRAM are initialized and the Src images
//! initialized
//!
//! The program operates in one of the two modes - as directed by preprocessor macro SWIPE
//! If SWIPE is defined, the program demonstrates vertical and horizontal scrolling of
//! the Src Images - solely using scatter-gather of the Source images - with no CPU based
//! composition.
//!
//! If SWIPE is not defined, the example demonstrates Compostion using CPU and Rendering
//! happen in parallel - using a Ping-Pong FrameBuffer.
//!
//! Composition:
//! Composition uses two source images, again in PSRAM.
//! Src1 & Src2 - both containing a small vertical bar.
//! Composition creates a sliding bar effect, with the Src1 and Src2 bars moving
//! in opposite directions, overlapped with each other. In addition the background
//! color is changed for each update as well.
//! Compositing is done in internal SRAM, with small fragments
//! of Src images brought in to SRAM from PSRAM using DMA.
//! CPU computes the final image fragment using these two Src image fragments
//! brought into SRAM, which is then DMA'ed to PSRAM.
//! Compile time modes allow direct composition in PSRAM using XIPMM (or a combination
//! of XIP and XIPMM) as well.
//! The process repeats till the whole image is constructed.
//!
//! Rendering:
//! This example demonstrates transferring a large buffer from a PSRAM device
//! connected on MSPI, to a Display device connected to another MSPI, using
//! hardware handshaking in Apollo3 Blue Plus - with minimal CPU involvement.
//! MSPI PSRAM imposes limits on max transaction size as well as page size
//! restrictions which need to be accounted for each MSPI transcation. Most of
//! these are handled by the Apollo3 Blue Plus hardware itself.
//! For the hardware based transaction splitting to work though, the address
//! needs to be word aligned.  Hence the SW still takes care of some splitting
//! of transactions, if needed.
//!
//! The program here creates a command queue for both the MSPIs, to
//! create a sequence of transactions - each reading a segment of the source
//! buffer to a temp buffer in internal SRAM, and then writing the same to the
//! Display. It uses hardware handshaking so that the Display transaction
//! is started only once the segement is read out completely from MSPI PSRAM.
//! To best utilize the buses, a ping-pong model is used using two temporary
//! buffers in SRAM. This allows the interfaces to not idle while waiting for
//! other to finish - essentially achieving close to the bandwidth achieved by
//! the slower of the two.
//! Rendering is synchronized with the TE signal from the display to ensure
//! no tearing on the display as the display buffer is being updated
//!
//! XIP:
//! If enabled, a timer is started to run a prime function out of PSRAM periodically
//!
//! XIPMM:
//! If enabled, a timer is started to run a demo function to exercise XIPMM out of PSRAM
//!
//! Configurable parameters at compile time:
//! MSPI to use for DISPLAY  (DISPLAY_MSPI_MODULE)
//! MSPI to use for PSRAM  (PSRAM_MSPI_MODULE)
//!
//! Operating modes:
//! CQ_RAW - Uses Preconstructed CQ (only small changes done at
//! run time) - to save on the time to program the same at run time
//!
//! Memory Impacts of modes: If CQ_RAW is used - the buffer supplied to HAL for CQ could be very small,
//! as the raw CQ is supplied by the application. So overall memory usage is still about the same
//!
//! Independent Controls:
//! Composition Modes:
//!     How to Read Src Buffer: MODE_SRCBUF_READ (DMA/XIP/XIPMM <Apollo3-B0 Only>)
//!     How to Write to Frame Buffer: MODE_DESTBUF_WRITE (DMA/XIPMM <Apollo3-B0 Only>)
//!
//! @verbatim
//! Pin connections:
//!
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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "freertos_mspi_mspi_display.h"
#include "rtos.h"


//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_itm_printf_enable();
}


//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clock frequency
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

#if 1
    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();
#endif

#ifndef NOFPU
    //
    // Enable the floating point module, and configure the core for lazy
    // stacking.
    //
    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);
#else
    am_hal_sysctrl_fpu_disable();
#endif

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // Enable printing to the console.
    //
#ifdef AM_DEBUG_PRINTF
    enable_print_interface();
#endif

    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("FreeRTOS MSPI-MSPI Display Example\n");


    //
    // Debug GPIO
    //
    am_hal_gpio_pinconfig(DBG1_GPIO, g_AM_HAL_GPIO_OUTPUT_12);
    DEBUG_GPIO_LOW(DBG1_GPIO);
    am_hal_gpio_pinconfig(DBG2_GPIO, g_AM_HAL_GPIO_OUTPUT_12);
    DEBUG_GPIO_LOW(DBG2_GPIO);

    //
    // Run the application.
    //
    run_tasks();

    //
    // We shouldn't ever get here.
    //
    while (1)
    {
    }

}

