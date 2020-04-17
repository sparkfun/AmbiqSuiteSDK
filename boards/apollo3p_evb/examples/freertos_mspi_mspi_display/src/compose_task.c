//*****************************************************************************
//
//! @file compose_task.c
//!
//! @brief Task to handle composition operation.
//!
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
#include "freertos_mspi_mspi_display.h"
#include "compose_task.h"

//*****************************************************************************
//
// Composition Fragments
//
//*****************************************************************************
// Control the granularity of composition
// For SWIPE - this also controls the size of temp buffer
#define NUM_ROW_PER_COMPOSE_FRAGMENT      2
#define COMPOSE_FRAGMENT_SIZE             (NUM_ROW_PER_COMPOSE_FRAGMENT * ROW_SIZE)
#define NUM_COMPOSE_FRAGMENTS             ((ROW_NUM + NUM_ROW_PER_COMPOSE_FRAGMENT - 1) / NUM_ROW_PER_COMPOSE_FRAGMENT)


//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile bool   g_bNewDisplay = false;
volatile bool   g_bNewFB = false;
void            *g_PsramHandle;
void            *g_MSPIPsramHandle;

uint32_t        fb1 = PSRAM_ACTFB1_OFFSET;
uint32_t        fb2 = PSRAM_ACTFB2_OFFSET;
#ifdef SWIPE
uint8_t         numSwipes = 0;
bool            bSwipeHorizontal = true;
uint32_t        fb2Offset = COLUMN_NUM;
#endif


//*****************************************************************************
//
// Local Variables
//
//*****************************************************************************
// Buffer for non-blocking transactions for Source MSPI - Needs to be big enough to accomodate
// all the transactions
#ifndef CQ_RAW
// A factor of 2 is needed because each transaction could be split into 2 to ensure the large
// transaction always starts at word aligned address
static uint32_t        g_MspiPsramQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (NUM_FRAGMENTS + 1) * 2];
#else
static uint32_t        g_MspiPsramQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (2 + 1)];

#endif

#ifdef SWIPE
// Pre-constructed Image(s)
extern const unsigned char g_ambiq_logo_bmp[];
extern const unsigned char g_ambiq_logo_bmp0[];
extern const unsigned char g_ambiq_logo_bmp1[];
extern const unsigned char g_ambiq_logo_bmp2[];
extern const unsigned char g_ambiq_logo_bmp_allblack[];
uint8_t                    *img0 = (uint8_t *)g_ambiq_logo_bmp0;
uint8_t                    *img1 = (uint8_t *)g_ambiq_logo_bmp2;
//uint8_t                    *img0 = (uint8_t *)g_ambiq_logo_bmp_allblack;
//uint8_t                    *img1 = (uint8_t *)g_ambiq_logo_bmp_allblack;
#else
static uint32_t         g_RxFrag[2][COMPOSE_FRAGMENT_SIZE / 4];
static uint32_t         g_TxFrag[COMPOSE_FRAGMENT_SIZE / 4];
// Composition States: n is NUM_COMPOSE_FRAGMENTS
// 0: Initial State - Read two fragments
// 1: Compose frag # 0, Read Frag # 1
// 2: Compose frag # 1, Read Frag # 2
// 3: Compose frag # 2, Read Frag # 3
// 4: Compose frag # 3, Read Frag # 4
// n-1: Compose frag # n-1, Read Frag # 0
static uint32_t        g_ComposeState = 0;
static uint32_t        g_numCompose = 0;

// Memory for hiprio transactions
// Need to use hiprio transactions to insert in between a running sequence
static uint8_t         g_MspiHiPrioBuf[(AM_HAL_MSPI_HIPRIO_ENTRY_SIZE)*6]; // We write two lines - which could be split into 4 transactions
#endif

//*****************************************************************************
//
// Composition task handle.
//
//*****************************************************************************
TaskHandle_t compose_task_handle;

//*****************************************************************************
//
// Handle for Compose-related events.
//
//*****************************************************************************
EventGroupHandle_t xComposeEventHandle;

am_devices_mspi_psram_config_t MSPI_PSRAM_QuadCE0MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_QUAD_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
    .eMixedMode               = AM_HAL_MSPI_XIPMIXED_NORMAL,
    .ui32NBTxnBufLength       = sizeof(g_MspiPsramQBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = g_MspiPsramQBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
#ifdef DEBUG
uint32_t g_History[256];
uint8_t g_HistoryCount = 0;

#define ADD_HISTORY(hist)  \
do { \
  g_History[g_HistoryCount++] = (hist); \
} while(0)
#else

#define ADD_HISTORY(x)

#endif

//! MSPI interrupts.
static const IRQn_Type mspi_psram_interrupts[] =
{
    MSPI0_IRQn,
#if defined(AM_PART_APOLLO3P)
    MSPI1_IRQn,
    MSPI2_IRQn,
#endif
};

//
// Take over the interrupt handler for whichever MSPI we're using.
//
#define psram_mspi_isr                                                          \
    am_mspi_isr1(PSRAM_MSPI_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void psram_mspi_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIPsramHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIPsramHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIPsramHandle, ui32Status);
}

// Initialization
uint32_t
psram_init(void)
{
    uint32_t ui32Status;
    
    // Set up the MSPI configuraton for the N25Q256A part.
    ui32Status = am_devices_mspi_psram_init(PSRAM_MSPI_MODULE,
                                            &MSPI_PSRAM_QuadCE0MSPIConfig,
                                            &g_PsramHandle,
                                            &g_MSPIPsramHandle);
    NVIC_EnableIRQ(mspi_psram_interrupts[PSRAM_MSPI_MODULE]);

    am_hal_interrupt_master_enable();
#ifndef SWIPE
    am_hal_mspi_hiprio_cfg_t mspiHiprioCfg;
    mspiHiprioCfg.pBuf = g_MspiHiPrioBuf;
    mspiHiprioCfg.size = sizeof(g_MspiHiPrioBuf);
    am_hal_mspi_control(g_MSPIPsramHandle, AM_HAL_MSPI_REQ_INIT_HIPRIO, &mspiHiprioCfg);
#if (MODE_SRCBUF_READ != MODE_DMA) || (MODE_DESTBUF_WRITE != MODE_DMA)
    //
    // Set up for XIP operation.
    //
    am_util_stdio_printf("Putting the MSPI and External PSRAM into XIP mode\n");
    ui32Status = am_devices_mspi_psram_enable_xip(g_PsramHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to put the MSPI into XIP mode!\n");
        return -1;
    }
#ifndef XIP_UNCACHED
    ui32Status = am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_FLASH_CACHE_INVALIDATE, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to invalidate Cache!\n");
        return -1;
    }
#else
    //  Mark XIP as non-cached - to make sure we see its impact
    am_hal_cachectrl_nc_cfg_t ncCfg;
    ncCfg.bEnable = true;
    ncCfg.eNCRegion = AM_HAL_CACHECTRL_NCR0;
    ncCfg.ui32StartAddr = PSRAM_XIP_BASE;
    ncCfg.ui32EndAddr = PSRAM_XIP_BASE + PSRAM_XIP_SIZE;
    ui32Status = am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_NC_CFG, &ncCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to mark XIP region as non-cacheable\n");
        return -1;
    }
#endif

#endif
#endif

    return ui32Status;
}

#ifdef VERIFY_SRCBUF
uint8_t rxBuf[TEMP_BUFFER_SIZE*4];
#endif

int
init_mspi_psram_data(void)
{
    uint32_t ui32Status;

    DEBUG_PRINT("Writing a known pattern to psram!\n");

#ifdef SWIPE
    ui32Status = am_devices_mspi_psram_write(g_PsramHandle, img0, fb1, FRAME_SIZE, true);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to write buffer to PSRAM Device!\n");
        return -1;
    }
#ifdef VERIFY_SRCBUF
    for (uint32_t address = 0; address < FRAME_SIZE; address += TEMP_BUFFER_SIZE)
    {
        //
        // Read the data back into the RX buffer.
        //
        ui32Status = am_devices_mspi_psram_read(g_PsramHandle, rxBuf, (uint32_t)(address + fb1), TEMP_BUFFER_SIZE, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to read buffer to PSRAM Device!\n");
            return -1;
        }

        uint32_t size = ((address + TEMP_BUFFER_SIZE) > FRAME_SIZE) ? (FRAME_SIZE - address) : TEMP_BUFFER_SIZE;
        //
        // Compare the buffers
        //
        for (uint32_t i = 0; i < size; i++)
        {
            if (rxBuf[i] != img0[i + address])
            {
                DEBUG_PRINT("TX and RX buffers failed to compare!\n");
                return -1;
            }
        }
    }
#endif

    ui32Status = am_devices_mspi_psram_write(g_PsramHandle, img1, fb2, FRAME_SIZE, true);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to write buffer to PSRAM Device!\n");
        return -1;
    }
#ifdef VERIFY_SRCBUF
    for (uint32_t address = 0; address < FRAME_SIZE; address += TEMP_BUFFER_SIZE)
    {
        //
        // Read the data back into the RX buffer.
        //
        ui32Status = am_devices_mspi_psram_read(g_PsramHandle, rxBuf, (uint32_t)(address + fb2), TEMP_BUFFER_SIZE, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to read buffer to PSRAM Device!\n");
            return -1;
        }

        uint32_t size = ((address + TEMP_BUFFER_SIZE) > FRAME_SIZE) ? (FRAME_SIZE - address) : TEMP_BUFFER_SIZE;
        //
        // Compare the buffers
        //
        for (uint32_t i = 0; i < size; i++)
        {
            if (rxBuf[i] != img1[i + address])
            {
                DEBUG_PRINT("TX and RX buffers failed to compare!\n");
                return -1;
            }
        }
    }
#endif
#else
    uint32_t i, j;
    //
    // Generate raw color data into PSRAM frame buffer 1
    //
    for (i = 0; i < NUM_ROW_PER_COMPOSE_FRAGMENT; i++)
    {
        uint32_t rowStart = i*(COLUMN_NUM / 4);
        for (j = 0; j < (COLUMN_NUM / 4); j++)
        {
            if (j < BAND_WIDTH / 4)
            {
                g_RxFrag[0][rowStart + j] = COLOR_4P(BAND_COLOR);
            }
            else
            {
                g_RxFrag[0][rowStart + j] = COLOR_4P(0xFF);
            }
            if (j >= (COLUMN_NUM - BAND_WIDTH) / 4)
            {
                g_RxFrag[1][rowStart + j] = COLOR_4P(COLOR_MAX ^ BAND_COLOR);
            }
            else
            {
                g_RxFrag[1][rowStart + j] = COLOR_4P(0xFF);
            }
        }
    }

    // Initialize Src Buf 1
    for (uint32_t address = 0; address < FRAME_SIZE; address += COMPOSE_FRAGMENT_SIZE)
    {
        uint32_t size = ((address + COMPOSE_FRAGMENT_SIZE) > FRAME_SIZE) ? (FRAME_SIZE - address) : COMPOSE_FRAGMENT_SIZE;
        //
        // Write the buffer into the target address in PSRAM
        //
        ui32Status = am_devices_mspi_psram_write(g_PsramHandle, (uint8_t *)g_RxFrag[0], PSRAM_SRCFB1_OFFSET + address, size, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to write buffer to PSRAM Device!\n");
            return -1;
        }
    }

    // Initialize Src Buf 2
    for (uint32_t address = 0; address < FRAME_SIZE; address += COMPOSE_FRAGMENT_SIZE)
    {
        uint32_t size = ((address + COMPOSE_FRAGMENT_SIZE) > FRAME_SIZE) ? (FRAME_SIZE - address) : COMPOSE_FRAGMENT_SIZE;
        //
        // Write the buffer into the target address in PSRAM
        //
        ui32Status = am_devices_mspi_psram_write(g_PsramHandle, (uint8_t *)g_RxFrag[1], PSRAM_SRCFB2_OFFSET + address, size, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to write buffer to PSRAM Device!\n");
            return -1;
        }
    }
    // Initialize ActFB1
    for (uint32_t address = 0; address < FRAME_SIZE; address += COMPOSE_FRAGMENT_SIZE)
    {
        uint32_t size = ((address + COMPOSE_FRAGMENT_SIZE) > FRAME_SIZE) ? (FRAME_SIZE - address) : COMPOSE_FRAGMENT_SIZE;
        //
        // Write the buffer into the target address in PSRAM
        //
        ui32Status = am_devices_mspi_psram_write(g_PsramHandle, (uint8_t *)g_RxFrag[0], PSRAM_ACTFB1_OFFSET + address, size, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to write buffer to PSRAM Device!\n");
            return -1;
        }
    }
    // Prepare the Fragments for the 1st iteration - already accomplished above, as g_RxFrag[] is already initialized
    g_ComposeState = 1;
#if (MODE_SRCBUF_READ == MODE_XIP)
#ifndef SRCBUF_XIP_UNCACHED
    ui32Status = am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_FLASH_CACHE_INVALIDATE, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to invalidate Cache!\n");
        return -1;
    }
#else
    //  Mark XIP SRCBUF as non-cached - to make sure we see its impact
    am_hal_cachectrl_nc_cfg_t ncCfg;
    ncCfg.bEnable = true;
    ncCfg.eNCRegion = AM_HAL_CACHECTRL_NCR1;
    ncCfg.ui32StartAddr = PSRAM_SRCFB_BASE;
    ncCfg.ui32EndAddr = PSRAM_SRCFB_BASE + PSRAM_SRCFB_SIZE;
    ui32Status = am_hal_cachectrl_control(AM_HAL_CACHECTRL_CONTROL_NC_CFG, &ncCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to mark XIP SRCBUF region as non-cacheable\n");
        return -1;
    }
#endif
#endif
#endif
    return 0;
}
//*****************************************************************************
//
// Perform initial setup for the compose task.
//
//*****************************************************************************
void
ComposeTaskSetup(void)
{
    uint32_t iRet;
    am_util_debug_printf("ComposeTask: setup\r\n");
    //
    // Create an event handle for our wake-up events.
    //
    xComposeEventHandle = xEventGroupCreate();

    //
    // Make sure we actually allocated space for the events we need.
    //
    while (xComposeEventHandle == NULL);
    // Initialize psram Data
    iRet = init_mspi_psram_data();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize MSPI psram data\n");
        while(1);
    }
    // Notify compose task that the frame is ready
    xEventGroupSetBits(xComposeEventHandle, COMPOSE_EVENT_START_NEW_FRAME);
}

#ifndef SWIPE

void
frag_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("\nCompose# %d:Fragment %d Failed 0x%x\n", g_numCompose, g_ComposeState, transactionStatus);
    }
    else
    {
        DEBUG_PRINT_SUCCESS("\nCompose# %d:Fragment %d Done 0x%x\n", g_numCompose, g_ComposeState, transactionStatus);
        g_ComposeState++;
    }
    //
    // Send an event to the main LED task
    //
    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(xComposeEventHandle, COMPOSE_EVENT_NEW_FRAG_READY,
                                        &xHigherPriorityTaskWoken);

    //
    // If the LED task is higher-priority than the context we're currently
    // running from, we should yield now and run the radio task.
    //
    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

// Compose the next fragment
static void
composeFragment(uint32_t *pSrcBuf1, uint32_t *pSrcBuf2, uint32_t *pDestBuf, uint32_t numRows)
{
    uint32_t i, j;
    // Compute the output FB fragment
    for (i = 0; i < numRows; i++)
    {
        uint32_t rowStart = i*(COLUMN_NUM / 4);
        for (j = 0; j < (COLUMN_NUM / 4); j++)
        {
            if (pSrcBuf2[rowStart + (j + g_numCompose) % (COLUMN_NUM / 4)] != COLOR_4P(0xFF))
            {
                if (pSrcBuf1[rowStart + ((j + (COLUMN_NUM / 4) - g_numCompose % (COLUMN_NUM / 4)) % (COLUMN_NUM / 4))] != COLOR_4P(0xFF))
                {
                    // Overlap region
                    pDestBuf[rowStart + j] = COLOR_4P(COLOR_MAX ^ (g_numCompose % COLOR_MAX));
                }
                else
                {
                    // Only Src2
                    pDestBuf[rowStart + j] = pSrcBuf2[rowStart + (j + g_numCompose) % (COLUMN_NUM / 4)];
                }
            }
            else
            {
                if (pSrcBuf1[rowStart + ((j + (COLUMN_NUM / 4) - g_numCompose % (COLUMN_NUM / 4)) % (COLUMN_NUM / 4))] != COLOR_4P(0xFF))
                {
                    // Only Src1
                    pDestBuf[rowStart + j] = pSrcBuf1[rowStart + ((j + (COLUMN_NUM / 4) - g_numCompose % (COLUMN_NUM / 4)) % (COLUMN_NUM / 4))];
                }
                else
                {
                    // None
                    pDestBuf[rowStart + j] = COLOR_4P(g_numCompose % COLOR_MAX);
                }
            }
        }
    }
}

// Simulate the composition task using a state machine
// The composition is done in piecemeal - one fragment at a time
// Each step includes: Compose the output fragment. Write to Destibation FB PSRAM. Preload Src Fragments from PSRAM
uint32_t
compose(void)
{
    static uint32_t lastState = 0;
    uint32_t ui32Status;
    uint32_t *pSrcBuf1;
    uint32_t *pSrcBuf2;
    uint32_t *pDestBuf;

    uint32_t actFbOffset = (g_numCompose & 0x1) ? PSRAM_ACTFB2_OFFSET: PSRAM_ACTFB1_OFFSET;


#if (MODE_SRCBUF_READ == MODE_XIP)
    pSrcBuf1 = (uint32_t *)(MSPI_XIP_BASEADDRn(PSRAM_MSPI_MODULE) + PSRAM_SRCFB1_OFFSET + (g_ComposeState - 1)*COMPOSE_FRAGMENT_SIZE);
    pSrcBuf2 = (uint32_t *)(MSPI_XIP_BASEADDRn(PSRAM_MSPI_MODULE) + PSRAM_SRCFB2_OFFSET + (g_ComposeState - 1)*COMPOSE_FRAGMENT_SIZE);
#elif (MODE_SRCBUF_READ == MODE_XIPMM)
    pSrcBuf1 = (uint32_t *)(MSPI_XIPMM_BASEADDRn(PSRAM_MSPI_MODULE) + PSRAM_SRCFB1_OFFSET + (g_ComposeState - 1)*COMPOSE_FRAGMENT_SIZE);
    pSrcBuf2 = (uint32_t *)(MSPI_XIPMM_BASEADDRn(PSRAM_MSPI_MODULE) + PSRAM_SRCFB2_OFFSET + (g_ComposeState - 1)*COMPOSE_FRAGMENT_SIZE);
#else
    pSrcBuf1 = g_RxFrag[0];
    pSrcBuf2 = g_RxFrag[1];
#endif

#if (MODE_DESTBUF_WRITE == MODE_XIPMM)
    pDestBuf = (uint32_t *)(MSPI_XIPMM_BASE_ADDRn(PSRAM_MSPI_MODULE) + actFbOffset + lastState*COMPOSE_FRAGMENT_SIZE);
#else
    pDestBuf = (uint32_t *)g_TxFrag;
#endif

    if ((g_ComposeState != lastState) && g_bNewFB)
    {
        composeFragment(pSrcBuf1, pSrcBuf2, pDestBuf, NUM_ROW_PER_COMPOSE_FRAGMENT);
        if (xSemaphoreTake(xMSPIMutex, portMAX_DELAY) != pdTRUE)
        {
            am_util_stdio_printf("Failed to get the semaphore\n");
            return AM_HAL_STATUS_FAIL;
        }
        // Write the fragment to next frame buffer
#if (MODE_DESTBUF_WRITE != MODE_XIPMM)
#if (MODE_SRCBUF_READ == MODE_DMA)
        ui32Status = am_devices_mspi_psram_write_hiprio(g_PsramHandle, (uint8_t *)g_TxFrag, actFbOffset + lastState*COMPOSE_FRAGMENT_SIZE, COMPOSE_FRAGMENT_SIZE, NULL, 0);
#else
        ui32Status = am_devices_mspi_psram_write_hiprio(g_PsramHandle, (uint8_t *)g_TxFrag, actFbOffset + lastState*COMPOSE_FRAGMENT_SIZE, COMPOSE_FRAGMENT_SIZE, frag_complete, 0);
#endif
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to write buffer to PSRAM Device!\n");
            xSemaphoreGive(xMSPIMutex);
            return ui32Status;
        }
#endif

        if (g_ComposeState == NUM_COMPOSE_FRAGMENTS)
        {
            // This FB is ready - now we can start for the next one
            g_ComposeState = 0;
        }
        lastState = g_ComposeState;
#if (MODE_SRCBUF_READ == MODE_DMA)
        // Read the two base fragments
        ui32Status = am_devices_mspi_psram_read_hiprio(g_PsramHandle, (uint8_t *)pSrcBuf2, PSRAM_SRCFB2_OFFSET + g_ComposeState*COMPOSE_FRAGMENT_SIZE, COMPOSE_FRAGMENT_SIZE, NULL, 0);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
            xSemaphoreGive(xMSPIMutex);
            return ui32Status;
        }
        ui32Status = am_devices_mspi_psram_read_hiprio(g_PsramHandle, (uint8_t *)pSrcBuf1, PSRAM_SRCFB1_OFFSET + g_ComposeState*COMPOSE_FRAGMENT_SIZE, COMPOSE_FRAGMENT_SIZE, frag_complete, 0);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
            xSemaphoreGive(xMSPIMutex);
            return ui32Status;
        }
#endif
        xSemaphoreGive(xMSPIMutex);
        if (g_ComposeState == 0)
        {
            DEBUG_PRINT("Done composing ActFB @ 0x%x\n", actFbOffset);
            g_bNewFB = false;
            g_numCompose++;
            // Notify main task that the frame is ready
            xEventGroupSetBits(xMainEventHandle, MAIN_EVENT_NEW_FRAME_READY);
#if (MODE_DESTBUF_WRITE == MODE_XIPMM) && (MODE_SRCBUF_READ != MODE_DMA)
            g_ComposeState++;
        }
        else
        {
            // No DMA - self signal completion
            g_ComposeState++;
            xEventGroupSetBits(xComposeEventHandle, COMPOSE_EVENT_NEW_FRAG_READY);
#endif
        }
    }
    return 0;
}
#endif

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
ComposeTask(void *pvParameters)
{
    uint32_t eventMask;

    while (1)
    {
        //
        // Wait for an event to be posted to the Compose Event Handle.
        //
        eventMask = xEventGroupWaitBits(xComposeEventHandle, 0xF, pdTRUE,
                            pdFALSE, portMAX_DELAY);
        if (eventMask != 0)
        {
            if (eventMask & COMPOSE_EVENT_START_NEW_FRAME)
            {
                g_bNewFB = true;
            }
            else
            {
            }
#ifdef SWIPE
            // No compose needed - just change the scatter gather settings for rendering
            fb2Offset -= 4;
            if (fb2Offset == 0)
            {
                if (++numSwipes == MAX_SWIPES)
                {
                    bSwipeHorizontal = !bSwipeHorizontal;
                    numSwipes = 0;
                }
                // Swap FB1 and FB2
                if (bSwipeHorizontal)
                {
                    fb2Offset = COLUMN_NUM;
                }
                else
                {
                    fb2Offset = ROW_NUM;
                }
                uint32_t temp = fb1;
                fb1 = fb2;
                fb2 = temp;
            }
            // No Compositing - Just indicate Frame Ready
            xEventGroupSetBits(xMainEventHandle, MAIN_EVENT_NEW_FRAME_READY);
#else
            if (compose())
            {
                while(1);
            }
#endif
        }
    }
}
