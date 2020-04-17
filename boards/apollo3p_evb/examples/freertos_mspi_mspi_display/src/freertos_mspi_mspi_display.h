//*****************************************************************************
//
//! @file freertos_mspi_iom_display.h
//!
//! @brief Global includes for the freertos_mspi_iom_display example.
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

#ifndef FREERTOS_MSPI_MSPI_DISPLAY_H
#define FREERTOS_MSPI_MSPI_DISPLAY_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
//#include "rtos.h"

//*****************************************************************************
//
// Task include files.
//
//*****************************************************************************
#include "compose_task.h"
#include "main_task.h"
#include "render_task.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_mspi_rm67162.h"
#include "am_devices_mspi_psram_aps6404l.h"
#include "am_util.h"

//*****************************************************************************
// Customize the following for the test
//*****************************************************************************
//#define ENABLE_LOGGING
// To test with different FB alignment
#define FB_ALIGNMENT_OFFSET    1 // 0 - 3
// To test image sanity with with FPGA, as the refresh rate is too high
#define TE_DELAY               1 // 1
// Demonstrate a horizontal scroll just by CQ scatter-gather without doing any composition
#define SWIPE
// Change from Horizontal to Vertical swipe after so many scrolls
#define MAX_SWIPES             2

#define NUM_FB                 2
#define DISPLAY_MSPI_MODULE    AM_BSP_MSPI_DISPLAY_INST
#define PSRAM_MSPI_MODULE      AM_BSP_MSPI_PSRAM_INST


// Whether to set the display render window
// This example always uses full screen, and hence this can be disabled for optimization
//#define CONFIG_DISPLAY_WINDOW
#define MODE_DMA                0
#define MODE_XIP                1
#define MODE_XIPMM              2

// How to read the Src Buffers?
// We could read it into the internal SRAM using the DMA, or use the XIP apperture
// to use it directly
//#define MODE_SRCBUF_READ        MODE_DMA
//#define MODE_SRCBUF_READ        MODE_XIP
//#define SRCBUF_XIP_UNCACHED
//#define MODE_SRCBUF_READ        MODE_XIPMM

// How to write the Dest Buffers?
// We could write it into the internal SRAM and then use the DMA to write to PSRAM, or use the XIPMM apperture
// to use it directly
//#define MODE_DESTBUF_WRITE      MODE_DMA
//#define MODE_DESTBUF_WRITE      MODE_XIPMM

#ifndef MODE_DESTBUF_WRITE
#define MODE_DESTBUF_WRITE        MODE_DMA
#endif

#ifndef MODE_SRCBUF_READ
#define MODE_SRCBUF_READ          MODE_DMA
#endif

// Build Raw CQ instead of using driver APIs to construct command queue sequence
#define CQ_RAW

// For Swipe - we need to use CQ_RAW
#if defined(SWIPE) && !defined(CQ_RAW)
#define CQ_RAW
#endif


// Size of temporary buffer being used
// Ideally this should be as large as possible for efficiency
// The impact on memory could be either ways - as increasing this increases the temp buf size
// however, at the same time - it reduces the memory required for command queue, as there are less
// number of transactions needed to transfer full frame buffer
#define TEMP_BUFFER_SIZE          4096
// Total number of TEMP_BUFFER_SIZE fragments needs to transfer one full frame buffer
#define NUM_FRAGMENTS             ((FRAME_SIZE + TEMP_BUFFER_SIZE - 1) / TEMP_BUFFER_SIZE)


//*****************************************************************************
//
// Macros
//
//*****************************************************************************
#define DISPLAY_MSPI_IRQn        MSPI_IRQn(DISPLAY_MSPI_MODULE)
#define PSRAM_MSPI_IRQn          MSPI_IRQn(PSRAM_MSPI_MODULE)

// Display Characteristics
#define ROW_NUM                   AM_DEVICES_RM67162_NUM_ROWS
#define COLUMN_NUM                AM_DEVICES_RM67162_NUM_COLUMNS
#define PIXEL_SIZE                AM_DEVICES_RM67162_PIXEL_SIZE
#define ROW_SIZE                  (COLUMN_NUM * PIXEL_SIZE)
#define FRAME_SIZE                ROW_NUM * ROW_SIZE // Display device is limited to its resolution ratio// Temp Buffer in SRAM to read PSRAM data to, and write DISPLAY data from


// PSRAM space is divided into following sections for this experiment
// FB_Src - this is where we'll initialize Source Frame Buffer Images - Base1 & Base2
// FB - Active Frame buffers - FB1 & FB2
#define PSRAM_BASE            MSPI_XIP_BASEADDRn(PSRAM_MSPI_MODULE)
#define PSRAM_SRCFB_BASE      PSRAM_BASE
#define PSRAM_SRCFB_OFFSET    (PSRAM_SRCFB_BASE - PSRAM_BASE)
#define PSRAM_SRCFB1_OFFSET   (PSRAM_SRCFB_OFFSET)
#define PSRAM_SRCFB2_OFFSET   (((PSRAM_SRCFB1_OFFSET + FRAME_SIZE + PSRAM_PAGE_SIZE - 1)/PSRAM_PAGE_SIZE)*PSRAM_PAGE_SIZE)
#define PSRAM_SRCFB_SIZE      (1024*1024)
#define PSRAM_ACTFB_BASE      (PSRAM_SRCFB_SIZE + PSRAM_SRCFB_BASE)
#define PSRAM_ACTFB_OFFSET    (PSRAM_ACTFB_BASE - PSRAM_BASE)
#define PSRAM_ACTFB1_OFFSET   (PSRAM_ACTFB_OFFSET  + FB_ALIGNMENT_OFFSET)
#define PSRAM_ACTFB2_OFFSET   (((PSRAM_ACTFB1_OFFSET + FRAME_SIZE + PSRAM_PAGE_SIZE - 1)/PSRAM_PAGE_SIZE)*PSRAM_PAGE_SIZE  + FB_ALIGNMENT_OFFSET)
#define PSRAM_ACTFB_SIZE      (1024*1024)
#define PSRAM_PAGE_SIZE       1024

//*****************************************************************************
//
// Debug
//
//*****************************************************************************
#define DBG1_GPIO          33
#define DBG2_GPIO          32

#define DEBUG_GPIO_HIGH(gpio)   am_hal_gpio_state_write(gpio, AM_HAL_GPIO_OUTPUT_SET)
#define DEBUG_GPIO_LOW(gpio)    am_hal_gpio_state_write(gpio, AM_HAL_GPIO_OUTPUT_CLEAR)

#ifdef ENABLE_LOGGING
#define DEBUG_PRINT am_util_stdio_printf
#else
#define DEBUG_PRINT(...)
#endif
#define DEBUG_PRINT_SUCCESS(...)

#define RENDER_EVENT_START_NEW_FRAME        0x1
#define COMPOSE_EVENT_START_NEW_FRAME       0x1
#define COMPOSE_EVENT_NEW_FRAG_READY        0x2
//*****************************************************************************
//
// Global externs
//
//*****************************************************************************
// Temp Buffer in SRAM to read PSRAM data to, and write DISPLAY data from
extern uint32_t              g_TempBuf[2][TEMP_BUFFER_SIZE / 4];
extern void                  *g_MSPIDisplayHandle;
extern void                  *g_MSPIPsramHandle;

extern volatile bool         g_bTEInt;
extern volatile bool         g_bDisplayDone;
extern volatile bool         g_bNewDisplay;
extern volatile uint32_t     g_actFb;

extern volatile bool         g_bNewFB;
extern uint32_t              fb1;
extern uint32_t              fb2;

#ifdef SWIPE
extern uint32_t              fb2Offset;
extern bool                  bSwipeHorizontal;
#endif


extern EventGroupHandle_t xMainEventHandle;
extern EventGroupHandle_t xComposeEventHandle;
extern EventGroupHandle_t xRenderEventHandle;
extern SemaphoreHandle_t xMSPIMutex;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
uint32_t display_init(void);

uint32_t psram_init(void);


#endif // FREERTOS_MSPI_MSPI_DISPLAY_H
