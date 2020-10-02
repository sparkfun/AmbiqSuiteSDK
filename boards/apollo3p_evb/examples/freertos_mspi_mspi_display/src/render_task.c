//*****************************************************************************
//
//! @file render_task.c
//!
//! @brief Task to handle rendering operation.
//!
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
#include "freertos_mspi_mspi_display.h"

//*****************************************************************************
//
// MSPI <==> MSPI Handshake Signals
//
//*****************************************************************************
#define MSPI_SIGNAL_SOURCE_BUFFER0      (MSPI0_CQFLAGS_CQFLAGS_SWFLAG0 << 8)
#define MSPI_SIGNAL_SOURCE_BUFFER1      (MSPI0_CQFLAGS_CQFLAGS_SWFLAG1 << 8)
#define MSPI_SIGNAL_SINK_BUFFER0        (MSPI0_CQFLAGS_CQFLAGS_SWFLAG0 << 8)
#define MSPI_SIGNAL_SINK_BUFFER1        (MSPI0_CQFLAGS_CQFLAGS_SWFLAG1 << 8)

#define MSPI_WAIT_FOR_SOURCE_BUFFER0    (_VAL2FLD(MSPI0_CQPAUSE_CQMASK, MSPI0_CQPAUSE_CQMASK_BUF0XOREN))
#define MSPI_WAIT_FOR_SOURCE_BUFFER1    (_VAL2FLD(MSPI0_CQPAUSE_CQMASK, MSPI0_CQPAUSE_CQMASK_BUF1XOREN))
#define MSPI_WAIT_FOR_SINK_BUFFER0      (_VAL2FLD(MSPI0_CQPAUSE_CQMASK, MSPI0_CQPAUSE_CQMASK_IOM0READY))
#define MSPI_WAIT_FOR_SINK_BUFFER1      (_VAL2FLD(MSPI0_CQPAUSE_CQMASK, MSPI0_CQPAUSE_CQMASK_IOM1READY))

//*****************************************************************************
//
// Render Fragments
//
//*****************************************************************************
// Control the granularity of composition
// For SWIPE - this also controls the size of temp buffer
#define NUM_ROW_PER_RENDER_FRAGMENT     (TEMP_BUFFER_SIZE / ROW_SIZE)
#define RENDER_FRAGMENT_SIZE            (NUM_ROW_PER_RENDER_FRAGMENT * ROW_SIZE)
#define NUM_RENDER_FRAGMENTS            ((ROW_NUM + NUM_ROW_PER_RENDER_FRAGMENT - 1) / NUM_ROW_PER_RENDER_FRAGMENT)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile bool         g_bDisplayDone = false;
volatile bool         g_bTEInt = false;
void                  *g_MSPIDisplayHandle;
void                  *g_DisplayHandle;

//*****************************************************************************
//
// Local Variables
//
//*****************************************************************************
// Buffer for non-blocking transactions for Display MSPI - Needs to be big enough to accomodate
// all the transactions
#ifndef CQ_RAW
static uint32_t        g_MspiDisplayQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (NUM_FRAGMENTS + 1)];
#else
static uint32_t        g_MspiDisplayQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (2 + 1)];
#endif

// Temp Buffer in SRAM to read PSRAM data to, and write DISPLAY data from
uint32_t        g_TempBuf[2][TEMP_BUFFER_SIZE / 4];

// Display MSPI configuration
static am_devices_mspi_rm67162_config_t SerialDisplayMSPICfg =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_48MHZ,
    .ui32NBTxnBufLength       = sizeof(g_MspiDisplayQBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = g_MspiDisplayQBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

//! MSPI interrupts.
static const IRQn_Type mspi_display_interrupts[] =
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
#define display_mspi_isr                                                          \
    am_mspi_isr1(DISPLAY_MSPI_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void display_mspi_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIDisplayHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIDisplayHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIDisplayHandle, ui32Status);
}

//*****************************************************************************
//
// Render task handle.
//
//*****************************************************************************
TaskHandle_t render_task_handle;

//*****************************************************************************
//
// Handle for Render-related events.
//
//*****************************************************************************
EventGroupHandle_t xRenderEventHandle;

uint32_t g_numDisplay = 0;

static void teInt_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    // Signal main task that TE has arrived
    static uint32_t teCount = 0;
    if (teCount++ < TE_DELAY)
    {
        return;
    }
    teCount = 0;
    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(xMainEventHandle, MAIN_EVENT_TE,
                                        &xHigherPriorityTaskWoken);

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Initialization
uint32_t
display_init(void)
{
    uint32_t ui32Status;

    NVIC_SetPriority(DISPLAY_MSPI_IRQn, 0x4);


    am_hal_gpio_pinconfig(AM_BSP_GPIO_DISPLAY_TE, g_AM_BSP_GPIO_DISPLAY_TE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_DISPLAY_RESET, g_AM_BSP_GPIO_DISPLAY_RESET);
    // Initialize the MSPI Display
    ui32Status = am_devices_mspi_rm67162_init(DISPLAY_MSPI_MODULE, &SerialDisplayMSPICfg, &g_DisplayHandle, &g_MSPIDisplayHandle);
    if (AM_DEVICES_RM67162_STATUS_SUCCESS != ui32Status)
    {
      DEBUG_PRINT("Failed to init Display device\n");
    }
    NVIC_EnableIRQ(mspi_display_interrupts[DISPLAY_MSPI_MODULE]);

    am_hal_interrupt_master_enable();
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear( AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_DISPLAY_TE));
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_DISPLAY_TE, teInt_handler);
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AM_BSP_GPIO_DISPLAY_TE));
    NVIC_EnableIRQ(GPIO_IRQn);

  return ui32Status;
}

void
psram_read_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        DEBUG_PRINT("\nPSRAM Read Failed 0x%x\n", transactionStatus);
    }
    else
    {
        DEBUG_PRINT("\nPSRAM Read Done 0x%x\n", transactionStatus);
    }
}

void
display_write_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        DEBUG_PRINT("\nDisplay# %d:FRAM Write Failed 0x%x\n", g_numDisplay, transactionStatus);
    }
    else
    {
        DEBUG_PRINT_SUCCESS("\nDisplay# %d:FRAM Write Done 0x%x\n", g_numDisplay, transactionStatus);
    }
    g_numDisplay++;
    // Signal main task that display is done
    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(xMainEventHandle, MAIN_EVENT_DISPLAY_DONE,
                                        &xHigherPriorityTaskWoken);

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

#ifdef CQ_RAW
// Jump - by reprogramming the CQADDR
typedef struct
{
    uint32_t    ui32CQAddrAddr;
    uint32_t    ui32CQAddrVal;
} am_hal_cq_jmp_t;

//
// Command Queue entry structure for DMA transfer.
//
typedef struct
{
    uint32_t                    ui32DMATARGADDRAddr;
    uint32_t                    ui32DMATARGADDRVal;
    uint32_t                    ui32DMADEVADDRAddr;
    // DEVADDR is don't care for display
    uint32_t                    ui32DMADEVADDRVal;
    uint32_t                    ui32DMATOTCOUNTAddr;
    uint32_t                    ui32DMATOTCOUNTVal;
    uint32_t                    ui32DMACFG1Addr;
    uint32_t                    ui32DMACFG1Val;
    // Need to insert couple of Dummy's
    uint32_t                    ui32DummyAddr1;
    uint32_t                    ui32DummyVal1;
    uint32_t                    ui32DummyAddr2;
    uint32_t                    ui32DummyVal2;
    // Need to disable the DMA to prepare for next reconfig
    // Need to have this following the DMAEN for CMDQ
    uint32_t                    ui32DMACFG2Addr;
    uint32_t                    ui32DMACFG2Val;
} mspi_cq_dma_entry_core_t;

//
// Command Queue entry structure for DMA transfer.
//
typedef struct
{
    uint32_t                    ui32PAUSENAddr;
    uint32_t                    ui32PAUSEENVal;
    uint32_t                    ui32PAUSEN2Addr;
    uint32_t                    ui32PAUSEEN2Val;
    mspi_cq_dma_entry_core_t    core;
    uint32_t                    ui32SETCLRAddr;
    uint32_t                    ui32SETCLRVal;
} mspi_cq_dma_entry_t;

typedef struct
{
    mspi_cq_dma_entry_core_t    core;
    am_hal_cq_jmp_t             jmp;
} mspi_cq_seg_t;

//
//
// Command Queue entry structure for DMA transfer.
//
typedef struct
{
    uint32_t                    ui32PAUSENAddr;
    uint32_t                    ui32PAUSEENVal;
    uint32_t                    ui32PAUSEN2Addr;
    uint32_t                    ui32PAUSEEN2Val;
    mspi_cq_seg_t               segment[NUM_ROW_PER_RENDER_FRAGMENT][4];
    uint32_t                    ui32SETCLRAddr;
    uint32_t                    ui32SETCLRVal;
} mspi_cq_frag_entry_t;

typedef struct
{
    mspi_cq_dma_entry_t       block[NUM_FRAGMENTS + 1];
    am_hal_cq_jmp_t           jmpOut; // Programmable address to jump back to the original CQ
} mspi_long_txn_t;

typedef struct
{
    mspi_cq_frag_entry_t      fragment[NUM_RENDER_FRAGMENTS];
    am_hal_cq_jmp_t           jmpOut; // Programmable address to jump back to the original CQ
} mspi_split_long_read_txn_t;

typedef struct
{
    mspi_cq_dma_entry_t       fragment[NUM_RENDER_FRAGMENTS];
    am_hal_cq_jmp_t           jmpOut; // Programmable address to jump back to the original CQ
} mspi_split_long_write_txn_t;

mspi_long_txn_t gMspiDisplayCQ;
mspi_long_txn_t gMspiPsramCQ;
mspi_split_long_read_txn_t gMspiPsramSplitLineCQ;
mspi_split_long_write_txn_t gMspiDisplaySplitLineCQ;

// MSPI
// One time initialization
void mspi_init_cq_long(uint32_t ui32Module, uint8_t ui8Priority, mspi_long_txn_t *pMspiLong, bool bSource, uint32_t blockSize)
{
    uint32_t ui32DmaCfg;
    if (bSource)
    {
        ui32DmaCfg =
            _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
            _VAL2FLD(MSPI0_DMACFG_DMAPRI, ui8Priority)    |
            _VAL2FLD(MSPI0_DMACFG_DMADIR, AM_HAL_MSPI_RX)     |
            _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
    }
    else
    {
        ui32DmaCfg =
            _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
            _VAL2FLD(MSPI0_DMACFG_DMAPRI, ui8Priority)    |
            _VAL2FLD(MSPI0_DMACFG_DMADIR, AM_HAL_MSPI_TX)     |
            _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
    }
    // Initialize the sequence blocks
    for (uint32_t i = 0; i < (NUM_FRAGMENTS + 1); i++)
    {
        pMspiLong->block[i].ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->block[i].ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->block[i].ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        pMspiLong->block[i].core.ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
        pMspiLong->block[i].core.ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
        pMspiLong->block[i].core.ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
        pMspiLong->block[i].core.ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        pMspiLong->block[i].core.ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        pMspiLong->block[i].core.ui32DummyAddr1 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        pMspiLong->block[i].core.ui32DummyAddr2 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        // Pause Conditions
        // This is the Pause Boundary for HiPrio transactions
        if (bSource)
        {
            pMspiLong->block[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((i % 2) ? MSPI_WAIT_FOR_SINK_BUFFER1 : MSPI_WAIT_FOR_SINK_BUFFER0);
            pMspiLong->block[i].ui32SETCLRVal = (i % 2) ? MSPI_SIGNAL_SINK_BUFFER1 : MSPI_SIGNAL_SINK_BUFFER0;
        }
        else
        {
            pMspiLong->block[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((i % 2) ? MSPI_WAIT_FOR_SOURCE_BUFFER1 : MSPI_WAIT_FOR_SOURCE_BUFFER0);
            pMspiLong->block[i].ui32SETCLRVal = (i % 2) ? MSPI_SIGNAL_SOURCE_BUFFER1 : MSPI_SIGNAL_SOURCE_BUFFER0;
        }
        pMspiLong->block[i].core.ui32DMATOTCOUNTVal = blockSize;
        pMspiLong->block[i].core.ui32DMATARGADDRVal = (i % 2) ? (uint32_t)&g_TempBuf[1] : (uint32_t)&g_TempBuf[0];
        pMspiLong->block[i].core.ui32DMACFG1Val = ui32DmaCfg;
        pMspiLong->block[i].core.ui32DMACFG2Val = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);
        pMspiLong->block[i].core.ui32DummyVal1 = 0;
        pMspiLong->block[i].core.ui32DummyVal2 = 0;
        pMspiLong->block[i].core.ui32DMADEVADDRVal = 0;
        pMspiLong->block[i].ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
    }
    pMspiLong->block[0].core.ui32DMATOTCOUNTVal = blockSize / 2;
    pMspiLong->block[1].core.ui32DMATOTCOUNTVal = blockSize / 2;
    pMspiLong->jmpOut.ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
}

#ifdef SWIPE
void mspi_init_split_read_cq_long(uint32_t ui32Module, uint8_t ui8Priority, mspi_split_long_read_txn_t *pMspiLong)
{
    uint32_t ui32DmaCfg;
    ui32DmaCfg =
        _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
        _VAL2FLD(MSPI0_DMACFG_DMAPRI, ui8Priority)    |
        _VAL2FLD(MSPI0_DMACFG_DMADIR, AM_HAL_MSPI_RX)     |
        _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
    // Initialize the sequence blocks
    for (uint32_t i = 0; i < NUM_RENDER_FRAGMENTS; i++)
    {
        pMspiLong->fragment[i].ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->fragment[i].ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->fragment[i].ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        // Pause Conditions
        // This is the Pause Boundary for HiPrio transactions
        pMspiLong->fragment[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((i % 2) ? MSPI_WAIT_FOR_SINK_BUFFER1 : MSPI_WAIT_FOR_SINK_BUFFER0);
        pMspiLong->fragment[i].ui32SETCLRVal = (i % 2) ? MSPI_SIGNAL_SINK_BUFFER1 : MSPI_SIGNAL_SINK_BUFFER0;
        pMspiLong->fragment[i].ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;

        for (uint32_t k = 0; k < NUM_ROW_PER_RENDER_FRAGMENT; k++)
        {
            for (uint32_t j = 0; j < 4; j++)
            {
                pMspiLong->fragment[i].segment[k][j].core.ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
                pMspiLong->fragment[i].segment[k][j].core.ui32DummyAddr1 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
                pMspiLong->fragment[i].segment[k][j].core.ui32DummyAddr2 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMACFG1Val = ui32DmaCfg;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMACFG2Val = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);
                pMspiLong->fragment[i].segment[k][j].core.ui32DummyVal1 = 0;
                pMspiLong->fragment[i].segment[k][j].core.ui32DummyVal2 = 0;
                pMspiLong->fragment[i].segment[k][j].core.ui32DMADEVADDRVal = 0;
                pMspiLong->fragment[i].segment[k][j].jmp.ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
            }
        }
    }
    // Special processing for last fragment
    if (ROW_NUM < (NUM_ROW_PER_RENDER_FRAGMENT * NUM_RENDER_FRAGMENTS))
    {
        uint32_t lastFragRows = ROW_NUM - (NUM_RENDER_FRAGMENTS - 1)*NUM_ROW_PER_RENDER_FRAGMENT;
        // Insert a jump to skip remaining row entries
        am_hal_cq_jmp_t *pJmp = (am_hal_cq_jmp_t *)&pMspiLong->fragment[NUM_RENDER_FRAGMENTS - 1].segment[lastFragRows];
        pJmp->ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
        pJmp->ui32CQAddrVal = (uint32_t)&pMspiLong->fragment[NUM_RENDER_FRAGMENTS - 1].segment[NUM_ROW_PER_RENDER_FRAGMENT];
    }
    pMspiLong->jmpOut.ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
}
#endif

// One time initialization
void mspi_init_split_write_cq_long(uint32_t ui32Module, uint8_t ui8Priority, mspi_split_long_write_txn_t *pMspiLong)
{
    uint32_t ui32DmaCfg;
    ui32DmaCfg =
        _VAL2FLD(MSPI0_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
        _VAL2FLD(MSPI0_DMACFG_DMAPRI, ui8Priority)    |
        _VAL2FLD(MSPI0_DMACFG_DMADIR, AM_HAL_MSPI_TX)     |
        _VAL2FLD(MSPI0_DMACFG_DMAEN, 3);
    // Initialize the sequence blocks
    for (uint32_t i = 0; i < NUM_RENDER_FRAGMENTS; i++)
    {
        pMspiLong->fragment[i].ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->fragment[i].ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        pMspiLong->fragment[i].ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        pMspiLong->fragment[i].core.ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
        pMspiLong->fragment[i].core.ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
        pMspiLong->fragment[i].core.ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
        pMspiLong->fragment[i].core.ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        pMspiLong->fragment[i].core.ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        pMspiLong->fragment[i].core.ui32DummyAddr1 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        pMspiLong->fragment[i].core.ui32DummyAddr2 = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        // Pause Conditions
        // This is the Pause Boundary for HiPrio transactions
        pMspiLong->fragment[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((i % 2) ? MSPI_WAIT_FOR_SOURCE_BUFFER1 : MSPI_WAIT_FOR_SOURCE_BUFFER0);
        pMspiLong->fragment[i].ui32SETCLRVal = (i % 2) ? MSPI_SIGNAL_SOURCE_BUFFER1 : MSPI_SIGNAL_SOURCE_BUFFER0;
        pMspiLong->fragment[i].core.ui32DMATOTCOUNTVal = ROW_SIZE * NUM_ROW_PER_RENDER_FRAGMENT;
        pMspiLong->fragment[i].core.ui32DMATARGADDRVal = (i % 2) ? (uint32_t)&g_TempBuf[1] : (uint32_t)&g_TempBuf[0];
        pMspiLong->fragment[i].core.ui32DMACFG1Val = ui32DmaCfg;
        pMspiLong->fragment[i].core.ui32DMACFG2Val = _VAL2FLD(MSPI0_DMACFG_DMAEN, 0);
        pMspiLong->fragment[i].core.ui32DummyVal1 = 0;
        pMspiLong->fragment[i].core.ui32DummyVal2 = 0;
        pMspiLong->fragment[i].core.ui32DMADEVADDRVal = 0;
        pMspiLong->fragment[i].ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
    }
    // Special processing for last fragment
    if (ROW_NUM < (NUM_ROW_PER_RENDER_FRAGMENT * NUM_RENDER_FRAGMENTS))
    {
        uint32_t lastFragRows = ROW_NUM - (NUM_RENDER_FRAGMENTS - 1)*NUM_ROW_PER_RENDER_FRAGMENT;
        pMspiLong->fragment[NUM_RENDER_FRAGMENTS - 1].core.ui32DMATOTCOUNTVal = ROW_SIZE * lastFragRows;
    }
    pMspiLong->jmpOut.ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
}

static void
update_mspi_mspi_transaction(uint32_t blockSize,
                             uint32_t ui32DevAddr,
                             mspi_long_txn_t *pMspiSrc,
                             mspi_long_txn_t *pMspiSink,
                             uint32_t ui32NumBytes)
{
    uint32_t headerSize = ui32DevAddr & 0x3;
    pMspiSrc->block[0].core.ui32DMADEVADDRVal = ui32DevAddr;
    if (headerSize)
    {
        headerSize = 4 - headerSize;
        // Special treatment needed as the address is not word aligned
        // We split the transaction in two - first just for the preceding bytes, and remaining in second one
        pMspiSrc->block[0].core.ui32DMATOTCOUNTVal = headerSize;
        pMspiSink->block[0].core.ui32DMATOTCOUNTVal = headerSize;
        ui32DevAddr += headerSize;
        pMspiSrc->block[1].core.ui32DMADEVADDRVal = ui32DevAddr;
        pMspiSrc->block[1].core.ui32DMATOTCOUNTVal = blockSize;
        pMspiSink->block[1].core.ui32DMATOTCOUNTVal = blockSize;
    }
    else
    {
        pMspiSrc->block[0].core.ui32DMATOTCOUNTVal = blockSize / 2;
        pMspiSink->block[0].core.ui32DMATOTCOUNTVal = blockSize / 2;
        pMspiSrc->block[1].core.ui32DMADEVADDRVal = ui32DevAddr + blockSize / 2;
        pMspiSrc->block[1].core.ui32DMATOTCOUNTVal = blockSize / 2;
        pMspiSink->block[1].core.ui32DMATOTCOUNTVal = blockSize / 2;
    }
    for (uint32_t i = 2; i < (NUM_FRAGMENTS + 1); i++)
    {
        pMspiSrc->block[i].core.ui32DMADEVADDRVal = ui32DevAddr + (i - 1) * blockSize;
    }
    // Initialize the count & command for tail
    if ((ui32NumBytes - headerSize) > (blockSize * (NUM_FRAGMENTS - 1)))
    {
        pMspiSrc->block[NUM_FRAGMENTS].core.ui32DMATOTCOUNTVal  = ui32NumBytes - headerSize - blockSize*(NUM_FRAGMENTS-1);
        pMspiSink->block[NUM_FRAGMENTS].core.ui32DMATOTCOUNTVal  = ui32NumBytes - headerSize - blockSize*(NUM_FRAGMENTS-1);
        pMspiSrc->block[NUM_FRAGMENTS-1].core.ui32DMATOTCOUNTVal = blockSize;
    }
    else
    {
        // Need to make sure there is non-zero last element
        // Adjust the second last
        pMspiSrc->block[NUM_FRAGMENTS-1].core.ui32DMATOTCOUNTVal = blockSize - 4;
        pMspiSrc->block[NUM_FRAGMENTS].core.ui32DMADEVADDRVal -= 4;
        pMspiSrc->block[NUM_FRAGMENTS].core.ui32DMATOTCOUNTVal  = ui32NumBytes - headerSize - blockSize*(NUM_FRAGMENTS-1) + 4;
        pMspiSink->block[NUM_FRAGMENTS].core.ui32DMATOTCOUNTVal  = ui32NumBytes - headerSize - blockSize*(NUM_FRAGMENTS-1) + 4;
    }
}

#ifdef SWIPE
static void
update_mspi_hsplit_transaction(uint32_t fb1, uint32_t fb2, uint32_t fb2ColOffset,
                               mspi_split_long_read_txn_t *pMspiSrc,
                               mspi_split_long_write_txn_t *pMspiSink)
{
    uint32_t headerSize1, headerSize2;
    uint32_t seg = 0;

    uint32_t fb1RowSize = fb2ColOffset*PIXEL_SIZE;
    uint32_t fb2RowSize = ROW_SIZE - fb2ColOffset*PIXEL_SIZE;
    headerSize1 = (fb1 & 0x3) ? (4 - (fb1 & 0x3)): 0;
    headerSize2 = (fb2 & 0x3) ? (4 - (fb2 & 0x3)): 0;
    // Offset in FB1
    fb1 = fb1 + ROW_SIZE - fb1RowSize;

    for (uint32_t i = 0; i < NUM_RENDER_FRAGMENTS; i++)
    {
        uint32_t bufAddr = (i % 2) ? (uint32_t)&g_TempBuf[1] : (uint32_t)&g_TempBuf[0];
        for (uint32_t k = 0; k < NUM_ROW_PER_RENDER_FRAGMENT; k++)
        {
            uint32_t remaining;
            uint32_t srcAddr;
            seg = 0;
            if (fb1RowSize)
            {
                remaining = fb1RowSize;
                srcAddr = fb1;
                if (headerSize1)
                {
                    uint32_t size = headerSize1;
                    if (headerSize1 > remaining)
                    {
                        size = remaining;
                    }
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = size;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                    pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                    remaining -= size;
                    srcAddr += size;
                    bufAddr += size;
                    seg++;
                }
                if (remaining)
                {
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = remaining;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                    pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                    seg++;
                    bufAddr += remaining;
                }
            }
            if (fb2RowSize)
            {
                remaining = fb2RowSize;
                srcAddr = fb2;
                if (headerSize2)
                {
                    uint32_t size = headerSize2;
                    if (headerSize2 > remaining)
                    {
                        size = remaining;
                    }
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = size;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                    pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                    remaining -= size;
                    srcAddr += size;
                    bufAddr += size;
                    seg++;
                }
                if (remaining)
                {
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = remaining;
                    pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                    pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                    seg++;
                    bufAddr += remaining;
                }
            }
            // Advance to end of segments
            pMspiSrc->fragment[i].segment[k][seg-1].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][4];
            fb1 += ROW_SIZE;
            fb2 += ROW_SIZE;
        }
    }
}

static void
update_mspi_vsplit_transaction(uint32_t fb1, uint32_t fb2, uint32_t fb2RowOffset,
                               mspi_split_long_read_txn_t *pMspiSrc,
                               mspi_split_long_write_txn_t *pMspiSink)
{
    uint32_t headerSize1, headerSize2, fb, headerSize;
    uint32_t seg = 0;

    headerSize1 = (fb1 & 0x3) ? (4 - (fb1 & 0x3)): 0;
    headerSize2 = (fb2 & 0x3) ? (4 - (fb2 & 0x3)): 0;

    // Offset in FB1
    fb = fb1 + ROW_SIZE * (ROW_NUM - fb2RowOffset);
    headerSize = headerSize1;

    for (uint32_t i = 0; i < NUM_RENDER_FRAGMENTS; i++)
    {
        uint32_t bufAddr = (i % 2) ? (uint32_t)&g_TempBuf[1] : (uint32_t)&g_TempBuf[0];
        for (uint32_t k = 0; k < NUM_ROW_PER_RENDER_FRAGMENT; k++)
        {
            uint32_t remaining;
            uint32_t srcAddr;
            seg = 0;
            remaining = ROW_SIZE;
            srcAddr = fb;
            if (headerSize)
            {
                uint32_t size = headerSize;
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = size;
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                remaining -= size;
                srcAddr += size;
                bufAddr += size;
                seg++;
            }
            if (remaining)
            {
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMADEVADDRVal = srcAddr;
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATOTCOUNTVal = remaining;
                pMspiSrc->fragment[i].segment[k][seg].core.ui32DMATARGADDRVal = bufAddr;
                pMspiSrc->fragment[i].segment[k][seg].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][seg + 1];
                seg++;
                bufAddr += remaining;
            }
            // Advance to end of segments
            pMspiSrc->fragment[i].segment[k][seg-1].jmp.ui32CQAddrVal = (uint32_t)&pMspiSrc->fragment[i].segment[k][4];
            if (--fb2RowOffset)
            {
                fb += ROW_SIZE;
            }
            else
            {
                fb = fb2;
                headerSize = headerSize2;
            }
        }
    }
}
#endif
#endif // CQ_RAW


// Initialization
uint32_t
init_mspi_mspi_xfer(void)
{
    uint32_t ui32Status = 0;
    uint32_t      u32Arg;

    if (xSemaphoreTake(xMSPIMutex, portMAX_DELAY) != pdTRUE)
    {
        goto _init_mspi_mspi_xfer_fail2;
    }
    // Clear flags
    u32Arg = AM_HAL_MSPI_SC_CLEAR(0xFF) & ~AM_HAL_MSPI_SC_RESV_MASK;  // clear all flags
    ui32Status = am_hal_mspi_control(g_MSPIPsramHandle, AM_HAL_MSPI_REQ_FLAG_SETCLR, &u32Arg);
    if (ui32Status)
    {
        goto _init_mspi_mspi_xfer_fail1;
    }

    ui32Status = am_hal_mspi_control(g_MSPIDisplayHandle, AM_HAL_MSPI_REQ_FLAG_SETCLR, &u32Arg);
    if (ui32Status)
    {
        goto _init_mspi_mspi_xfer_fail1;
    }

    // Link MSPI Source and Sink instances.
    u32Arg = DISPLAY_MSPI_MODULE;
    ui32Status = am_hal_mspi_control(g_MSPIPsramHandle, AM_HAL_MSPI_REQ_LINK_MSPI, &u32Arg);
    if (ui32Status)
    {
        goto _init_mspi_mspi_xfer_fail1;
    }

    u32Arg = PSRAM_MSPI_MODULE;
    ui32Status = am_hal_mspi_control(g_MSPIDisplayHandle, AM_HAL_MSPI_REQ_LINK_MSPI, &u32Arg);
    if (ui32Status)
    {
        goto _init_mspi_mspi_xfer_fail1;
    }

    ui32Status = am_devices_mspi_rm67162_set_transfer_window(g_DisplayHandle, 0, 0, ROW_NUM - 1, COLUMN_NUM - 1);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to set transfer window\n");
        goto _init_mspi_mspi_xfer_fail1;
    }
#ifdef CQ_RAW
#ifdef SWIPE
    mspi_init_split_read_cq_long(PSRAM_MSPI_MODULE, 1, &gMspiPsramSplitLineCQ);
    mspi_init_split_write_cq_long(DISPLAY_MSPI_MODULE, 1, &gMspiDisplaySplitLineCQ);
#else
    mspi_init_cq_long(PSRAM_MSPI_MODULE, 1, &gMspiPsramCQ, true, TEMP_BUFFER_SIZE);
    mspi_init_cq_long(DISPLAY_MSPI_MODULE, 1, &gMspiDisplayCQ, false, TEMP_BUFFER_SIZE);
#endif
#endif
_init_mspi_mspi_xfer_fail1:
    xSemaphoreGive(xMSPIMutex);
_init_mspi_mspi_xfer_fail2:
    return ui32Status;
}

// Rendering
uint32_t
start_mspi_mspi_xfer(uint32_t psramOffset, uint32_t ui32NumBytes)
{
    uint32_t      ui32Status = 0;
    am_hal_mspi_callback_t  mspiSourceCb = 0;
    am_hal_mspi_callback_t  mspiSinkCb = 0;

    //DEBUG_GPIO_HIGH(DBG1_GPIO);
    mspiSourceCb = psram_read_complete;
    mspiSinkCb = display_write_complete;
    DEBUG_PRINT("\nInitiating MSPI -> MSPI Transfer\n");
    if (xSemaphoreTake(xMSPIMutex, portMAX_DELAY) != pdTRUE)
    {
        return AM_HAL_STATUS_FAIL;
    }

#ifdef CONFIG_DISPLAY_WINDOW
    ui32Status = am_devices_mspi_rm67162_set_transfer_window(g_DisplayHandle, 0, 0, ROW_NUM - 1, COLUMN_NUM - 1);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to set transfer window\n");
        xSemaphoreGive(xMSPIMutex);
        return ui32Status;
    }
#endif

#ifdef CQ_RAW
    // Queue up the CQ Raw
    am_hal_cmdq_entry_t jump;
    am_hal_mspi_cq_raw_t rawMspiCfg;

    rawMspiCfg.ui32PauseCondition = 0;
    rawMspiCfg.ui32StatusSetClr = 0;
    rawMspiCfg.pCQEntry = &jump;
    rawMspiCfg.numEntries = sizeof(am_hal_cmdq_entry_t) / 8;
    rawMspiCfg.pCallbackCtxt = 0;

    update_mspi_mspi_transaction(TEMP_BUFFER_SIZE,
                                  psramOffset,
                                  &gMspiPsramCQ,
                                  &gMspiDisplayCQ,
                                  ui32NumBytes);

    rawMspiCfg.pJmpAddr = &gMspiPsramCQ.jmpOut.ui32CQAddrVal;
    jump.value = (uint32_t)&gMspiPsramCQ;
    rawMspiCfg.pfnCallback = mspiSourceCb;
    jump.address = (uint32_t)&MSPIn(PSRAM_MSPI_MODULE)->CQADDR;

    ui32Status = am_hal_mspi_control(g_MSPIPsramHandle, AM_HAL_MSPI_REQ_CQ_RAW, &rawMspiCfg);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to queue up MSPI Read transaction\n");
        while(1);
    }

    rawMspiCfg.pJmpAddr = &gMspiDisplayCQ.jmpOut.ui32CQAddrVal;
    jump.value = (uint32_t)&gMspiDisplayCQ;
    rawMspiCfg.pfnCallback = mspiSinkCb;
    jump.address = (uint32_t)&MSPIn(DISPLAY_MSPI_MODULE)->CQADDR;

    ui32Status = am_hal_mspi_control(g_MSPIDisplayHandle, AM_HAL_MSPI_REQ_CQ_RAW, &rawMspiCfg);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to queue up MSPI Write transaction\n");
        while(1);
    }
#else
    // Queue up Display Writes and PSRAM Reads
    for (uint32_t address = 0, bufIdx = 0; address < ui32NumBytes; address += TEMP_BUFFER_SIZE, bufIdx++)
    {
        uint32_t bufOdd = bufIdx % 2;
        ui32Status = am_devices_mspi_psram_read_adv(g_PsramHandle,
                                                    (uint8_t *)g_TempBuf[bufOdd],
                                                    psramOffset + address,
                                                    (((address + TEMP_BUFFER_SIZE) >= ui32NumBytes) ? (ui32NumBytes - address) : TEMP_BUFFER_SIZE),
                                                    (bufOdd ? MSPI_WAIT_FOR_SINK_BUFFER1 : MSPI_WAIT_FOR_SINK_BUFFER0),
                                                    (bufOdd ? MSPI_SIGNAL_SINK_BUFFER1 : MSPI_SIGNAL_SINK_BUFFER0),
                                                    (((address + TEMP_BUFFER_SIZE) >= ui32NumBytes) ? mspiSourceCb : 0),
                                                    0);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up MSPI Read transaction\n");
            break;
        }
        ui32Status = am_devices_rm67162_nonblocking_write_adv(g_DisplayHandle,
                                                   (uint8_t *)g_TempBuf[bufOdd],
                                                   (((address + TEMP_BUFFER_SIZE) >= ui32NumBytes) ? (ui32NumBytes - address) : TEMP_BUFFER_SIZE),
                                                   (bufOdd ? MSPI_WAIT_FOR_SOURCE_BUFFER1 : MSPI_WAIT_FOR_SOURCE_BUFFER0),
                                                   (bufOdd ? MSPI_SIGNAL_SOURCE_BUFFER1 : MSPI_SIGNAL_SOURCE_BUFFER0),
                                                   (((address + TEMP_BUFFER_SIZE) >= ui32NumBytes) ? mspiSinkCb : 0),
                                                   0);
        if (ui32Status)
        {
           DEBUG_PRINT("\nFailed to queue up MSPI Write transaction\n");
           break;
        }
    }
#endif
    xSemaphoreGive(xMSPIMutex);
    return ui32Status;
}

#ifdef SWIPE
uint32_t
start_split_mspi_mspi_xfer(uint32_t fb1, uint32_t fb2, uint32_t fb2Offset)
{
    uint32_t      ui32Status = 0;
    am_hal_mspi_callback_t  mspiSourceCb = 0;
    am_hal_mspi_callback_t  mspiSinkCb = 0;

    //DEBUG_GPIO_HIGH(DBG1_GPIO);
    mspiSourceCb = psram_read_complete;
    mspiSinkCb = display_write_complete;
    DEBUG_PRINT("\nInitiating MSPI -> MSPI Transfer\n");
    if (xSemaphoreTake(xMSPIMutex, portMAX_DELAY) != pdTRUE)
    {
        return AM_HAL_STATUS_FAIL;
    }

#ifdef CONFIG_DISPLAY_WINDOW
    ui32Status = am_devices_mspi_rm67162_set_transfer_window(g_DisplayHandle, 0, 0, ROW_NUM - 1, COLUMN_NUM - 1);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to set transfer window\n");
        xSemaphoreGive(xMSPIMutex);
        return ui32Status;
    }
#endif

    // Queue up the CQ Raw
    am_hal_cmdq_entry_t jump;
    am_hal_mspi_cq_raw_t rawMspiCfg;

    rawMspiCfg.ui32PauseCondition = 0;
    rawMspiCfg.ui32StatusSetClr = 0;
    rawMspiCfg.pCQEntry = &jump;
    rawMspiCfg.numEntries = sizeof(am_hal_cmdq_entry_t) / 8;
    rawMspiCfg.pCallbackCtxt = 0;

    if (bSwipeHorizontal)
    {
        update_mspi_hsplit_transaction(fb1, fb2, fb2Offset, &gMspiPsramSplitLineCQ, &gMspiDisplaySplitLineCQ);
    }
    else
    {
        update_mspi_vsplit_transaction(fb1, fb2, fb2Offset, &gMspiPsramSplitLineCQ, &gMspiDisplaySplitLineCQ);
    }

    rawMspiCfg.pJmpAddr = &gMspiPsramSplitLineCQ.jmpOut.ui32CQAddrVal;
    jump.value = (uint32_t)&gMspiPsramSplitLineCQ;
    rawMspiCfg.pfnCallback = mspiSourceCb;
    jump.address = (uint32_t)&MSPIn(PSRAM_MSPI_MODULE)->CQADDR;

    ui32Status = am_hal_mspi_control(g_MSPIPsramHandle, AM_HAL_MSPI_REQ_CQ_RAW, &rawMspiCfg);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to queue up MSPI Read transaction\n");
        while(1);
    }

    rawMspiCfg.pJmpAddr = &gMspiDisplaySplitLineCQ.jmpOut.ui32CQAddrVal;
    jump.value = (uint32_t)&gMspiDisplaySplitLineCQ;
    rawMspiCfg.pfnCallback = mspiSinkCb;
    jump.address = (uint32_t)&MSPIn(DISPLAY_MSPI_MODULE)->CQADDR;

    ui32Status = am_hal_mspi_control(g_MSPIDisplayHandle, AM_HAL_MSPI_REQ_CQ_RAW, &rawMspiCfg);
    if (ui32Status)
    {
        DEBUG_PRINT("\nFailed to queue up MSPI Write transaction\n");
        while(1);
    }
    xSemaphoreGive(xMSPIMutex);
    return ui32Status;
}
#endif

void
RenderTaskSetup(void)
{
    am_util_debug_printf("RenderTask: setup\r\n");
    //
    // Create an event handle for our wake-up events.
    //
    xRenderEventHandle = xEventGroupCreate();

    //
    // Make sure we actually allocated space for the events we need.
    //
    while (xRenderEventHandle == NULL);

    //
    // Run display
    //
    if (init_mspi_mspi_xfer())
    {
        while(1);
    }

#ifdef SWIPE
    if (start_split_mspi_mspi_xfer(fb1, fb2, fb2Offset))
    {
        while(1);
    }

#else
    if (start_mspi_mspi_xfer(fb1, FRAME_SIZE))
    {
        while(1);
    }

#endif
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RenderTask(void *pvParameters)
{
    uint32_t eventMask;

    while (1)
    {
        //
        // Wait for an event to be posted to the Radio Event Handle.
        //
        eventMask = xEventGroupWaitBits(xRenderEventHandle, 0xF, pdTRUE,
                            pdFALSE, portMAX_DELAY);
        if (eventMask != 0)
        {
            if (eventMask & RENDER_EVENT_START_NEW_FRAME)
            {
#ifdef SWIPE
                if ( start_split_mspi_mspi_xfer(fb1, fb2, fb2Offset) )
                {
                    while(1);
                }
#else
                uint32_t psramFbOffset;
                psramFbOffset = (g_numDisplay & 0x1) ? fb2: fb1;
                DEBUG_PRINT("Rendering ActFB %d\n", (g_numDisplay & 0x1));
                if (start_mspi_mspi_xfer(psramFbOffset, FRAME_SIZE))
                {
                    while(1);
                }
#endif
            }
        }
    }
}

