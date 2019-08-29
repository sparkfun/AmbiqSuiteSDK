//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief Task to handle radio operation.
//!
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2019, Ambiq Micro
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
// This is part of revision v2.2.0-7-g63f7c2ba1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "ble_freertos_amdtpc.h"

//*****************************************************************************
//
// WSF standard includes.
//
//*****************************************************************************
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_timer.h"

//*****************************************************************************
//
// Includes for operating the ExactLE stack.
//
//*****************************************************************************
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_drv_apollo3.h"

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"

#include "hci_apollo_config.h"
#include "wsf_msg.h"

//*****************************************************************************
//
// Includes for the AMDTP profile.
//
//*****************************************************************************
#include "amdtp_api.h"
#include "amdtpc_api.h"
#include "app_ui.h"

#ifdef BLE_MENU
#include "ble_menu.h"
#endif


//*****************************************************************************
//
// Radio task handle.
//
//*****************************************************************************
TaskHandle_t radio_task_handle;

//*****************************************************************************
//
// Handle for Radio-related events.
//
//*****************************************************************************
EventGroupHandle_t xRadioEventHandle;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void exactle_stack_init(void);
void scheduler_timer_init(void);
void update_scheduler_timers(void);
void set_next_wakeup(void);

//*****************************************************************************
//
// Timer configuration macros.
//
//*****************************************************************************
// Configure how to driver WSF Scheduler
#if 1
// Preferred mode to use when using FreeRTOS
#define USE_FREERTOS_TIMER_FOR_WSF
#else
// These are only test modes.
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
#define USE_STIMER_FOR_WSF // Reuse FreeRTOS used STimer for WSF
#else
#define USE_CTIMER_FOR_WSF
#endif
#endif

// Use FreeRTOS timer for WSF Ticks
#ifdef USE_FREERTOS_TIMER_FOR_WSF
#define CLK_TICKS_PER_WSF_TICKS     (WSF_MS_PER_TICK*configTICK_RATE_HZ/1000)
#endif

#ifdef USE_CTIMER_FOR_WSF
#define WSF_CTIMER_NUM              1
#define CLK_TICKS_PER_WSF_TICKS     (WSF_MS_PER_TICK*512/1000)   // Number of CTIMER counts per WSF tick.

#if WSF_CTIMER_NUM == 0
#define WSF_CTIMER_INT  AM_HAL_CTIMER_INT_TIMERA0
#elif WSF_CTIMER_NUM == 1
#define WSF_CTIMER_INT  AM_HAL_CTIMER_INT_TIMERA1
#elif WSF_CTIMER_NUM == 2
#define WSF_CTIMER_INT  AM_HAL_CTIMER_INT_TIMERA2
#elif WSF_CTIMER_NUM == 3
#define WSF_CTIMER_INT  AM_HAL_CTIMER_INT_TIMERA3
#endif
#endif

#ifdef USE_STIMER_FOR_WSF
#define CLK_TICKS_PER_WSF_TICKS     (WSF_MS_PER_TICK*configSTIMER_CLOCK_HZ/1000)   // Number of STIMER counts per WSF tick.
#endif

//*****************************************************************************
//
// WSF buffer pools.
//
//*****************************************************************************
#define WSF_BUF_POOLS               4

// Important note: the size of g_pui32BufMem should includes both overhead of internal
// buffer management structure, wsfBufPool_t (up to 16 bytes for each pool), and pool
// description (e.g. g_psPoolDescriptors below).

// Memory for the buffer pool
// extra AMOTA_PACKET_SIZE bytes for OTA handling
static uint32_t g_pui32BufMem[
        (WSF_BUF_POOLS*16
         + 16*8 + 32*4 + 64*6 + 280*8) / sizeof(uint32_t)];

// Default pool descriptor.
static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 280,  8 }
};

#ifdef BLE_MENU
wsfHandlerId_t g_uartDataReadyHandlerId;
void uart_data_ready_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    BleMenuRx();
}
#endif

//*****************************************************************************
//
// Tracking variable for the scheduler timer.
//
//*****************************************************************************
uint32_t g_ui32LastTime = 0;

void radio_timer_handler(void);

#ifdef USE_CTIMER_FOR_WSF
//*****************************************************************************
//
// Set up a pair of timers to handle the WSF scheduler.
//
//*****************************************************************************
void
scheduler_timer_init(void)
{
    // Enable the LFRC
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

    //
    // One of the timers will run in one-shot mode and provide interrupts for
    // scheduled events.
    //
    am_hal_ctimer_clear(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config_single(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA,
                                (AM_HAL_CTIMER_INT_ENABLE |
                                 AM_HAL_CTIMER_LFRC_512HZ |
                                 AM_HAL_CTIMER_FN_ONCE));

    //
    // The other timer will run continuously and provide a constant time-base.
    //
    am_hal_ctimer_clear(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERB);
    am_hal_ctimer_config_single(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERB,
                                (AM_HAL_CTIMER_LFRC_512HZ |
                                 AM_HAL_CTIMER_FN_CONTINUOUS));

    //
    // Start the continuous timer.
    //
    am_hal_ctimer_start(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERB);

    //
    // Enable the timer interrupt.
    //
    am_hal_ctimer_int_register(WSF_CTIMER_INT, radio_timer_handler);
    NVIC_SetPriority(CTIMER_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_hal_ctimer_int_enable(WSF_CTIMER_INT);
    NVIC_EnableIRQ(CTIMER_IRQn);
}

//*****************************************************************************
//
// Calculate the elapsed time, and update the WSF software timers.
//
//*****************************************************************************
void
update_scheduler_timers(void)
{
    uint32_t ui32CurrentTime, ui32ElapsedTime;

    //
    // Read the continuous timer.
    //
    ui32CurrentTime = am_hal_ctimer_read(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERB);

    //
    // Figure out how long it has been since the last time we've read the
    // continuous timer. We should be reading often enough that we'll never
    // have more than one overflow.
    //
    ui32ElapsedTime = ui32CurrentTime - g_ui32LastTime;

    //
    // Check to see if any WSF ticks need to happen.
    //
    if ( (ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS) > 0 )
    {
        //
        // Update the WSF timers and save the current time as our "last
        // update".
        //
        WsfTimerUpdate(ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS);

        g_ui32LastTime = ui32CurrentTime;
    }
}

//*****************************************************************************
//
// Set a timer interrupt for the next upcoming scheduler event.
//
//*****************************************************************************
void
set_next_wakeup(void)
{
    bool_t bTimerRunning;
    wsfTimerTicks_t xNextExpiration;

    //
    // Stop and clear the scheduling timer.
    //
    am_hal_ctimer_stop(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_clear(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA);

    //
    // Check to see when the next timer expiration should happen.
    //
    xNextExpiration = WsfTimerNextExpiration(&bTimerRunning);

    //
    // If there's a pending WSF timer event, set an interrupt to wake us up in
    // time to service it. Otherwise, set an interrupt to wake us up in time to
    // prevent a double-overflow of our continuous timer.
    //
    if ( xNextExpiration )
    {
        am_hal_ctimer_period_set(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA,
                                 xNextExpiration * CLK_TICKS_PER_WSF_TICKS, 0);
    }
    else
    {
        am_hal_ctimer_period_set(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA, 0x8000, 0);
    }

    //
    // Start the scheduling timer.
    //
    am_hal_ctimer_start(WSF_CTIMER_NUM, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Interrupt handler for the CTIMERs
//
//*****************************************************************************
void
radio_timer_handler(void)
{
    // Signal radio task to run

    WsfTaskSetReady(0, 0);
}
#endif
#ifdef USE_STIMER_FOR_WSF
//*****************************************************************************
//
// Reuse STIMER to handle the WSF scheduler.
//
//*****************************************************************************
void
scheduler_timer_init(void)
{
    //
    // USe CMPR1 of STIMER for one-shot mode interrupts for
    // scheduled events.
    //
    uint32_t cfgVal;
    /* Stop the Stimer momentarily.  */
    cfgVal = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);


    //
    // Configure the STIMER->COMPARE_1
    //
    am_hal_stimer_compare_delta_set(1, CLK_TICKS_PER_WSF_TICKS);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREB);
    am_hal_stimer_config(cfgVal | AM_HAL_STIMER_CFG_COMPARE_B_ENABLE);

    //
    // Enable the timer interrupt in the NVIC, making sure to use the
    // appropriate priority level.
    //
    NVIC_SetPriority(STIMER_CMPR1_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(STIMER_CMPR1_IRQn);

    //
    // Reuse STIMER to provide a constant time-base.
    //

}

//*****************************************************************************
//
// Calculate the elapsed time, and update the WSF software timers.
//
//*****************************************************************************
void
update_scheduler_timers(void)
{
    uint32_t ui32CurrentTime, ui32ElapsedTime;

    //
    // Read the continuous timer.
    //
    ui32CurrentTime = am_hal_stimer_counter_get();

    //
    // Figure out how long it has been since the last time we've read the
    // continuous timer. We should be reading often enough that we'll never
    // have more than one overflow.
    //
    ui32ElapsedTime = ui32CurrentTime - g_ui32LastTime;

    //
    // Check to see if any WSF ticks need to happen.
    //
    if ( (ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS) > 0 )
    {
        //
        // Update the WSF timers and save the current time as our "last
        // update".
        //
        WsfTimerUpdate(ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS);

        g_ui32LastTime = ui32CurrentTime;
    }
}

//*****************************************************************************
//
// Set a timer interrupt for the next upcoming scheduler event.
//
//*****************************************************************************
void
set_next_wakeup(void)
{
    bool_t bTimerRunning;
    wsfTimerTicks_t xNextExpiration;
    uint32_t cfgVal;
    uint32_t ui32Critical;

    //
    // Check to see when the next timer expiration should happen.
    //
    xNextExpiration = WsfTimerNextExpiration(&bTimerRunning);

    //
    // If there's a pending WSF timer event, set an interrupt to wake us up in
    // time to service it. Otherwise, set an interrupt to wake us up in time to
    // prevent a double-overflow of our continuous timer.
    //

    /* Enter a critical section */
    ui32Critical = am_hal_interrupt_master_disable();

    /* Stop the Stimer momentarily.  */
    cfgVal = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);
    //
    // Configure the STIMER->COMPARE_1
    //
    if ( xNextExpiration )
    {
        am_hal_stimer_compare_delta_set(1, xNextExpiration * CLK_TICKS_PER_WSF_TICKS);
    }
    else
    {
        am_hal_stimer_compare_delta_set(1, 0x80000000);
    }

    /* Exit Critical Section */
    am_hal_interrupt_master_set(ui32Critical);

    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREB);
    am_hal_stimer_config(cfgVal);

}

//*****************************************************************************
//
// Interrupt handler for the CTIMERs
//
//*****************************************************************************
void
radio_timer_handler(void)
{
    // Signal radio task to run

    WsfTaskSetReady(0, 0);
}

//*****************************************************************************
//
// Interrupt handler for the STIMER module Compare 1.
//
//*****************************************************************************
void
am_stimer_cmpr1_isr(void)
{
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);

    //
    // Run handlers for the various possible timer events.
    //
    radio_timer_handler();

}
#endif
#ifdef USE_FREERTOS_TIMER_FOR_WSF
TimerHandle_t xWsfTimer;
//*****************************************************************************
//
// Callback handler for the FreeRTOS Timer
//
//*****************************************************************************
void
wsf_timer_handler(TimerHandle_t xTimer)
{
    // Signal radio task to run

    WsfTaskSetReady(0, 0);
}

//*****************************************************************************
//
// Reuse FreeRTOS TIMER to handle the WSF scheduler.
//
//*****************************************************************************
void
scheduler_timer_init(void)
{
    // Create a FreeRTOS Timer
    xWsfTimer = xTimerCreate("WSF Timer", pdMS_TO_TICKS(WSF_MS_PER_TICK),
            pdFALSE, NULL, wsf_timer_handler);
    configASSERT(xWsfTimer);
}

//*****************************************************************************
//
// Calculate the elapsed time, and update the WSF software timers.
//
//*****************************************************************************
void
update_scheduler_timers(void)
{
    uint32_t ui32CurrentTime, ui32ElapsedTime;

    //
    // Read the continuous timer.
    //
    ui32CurrentTime = xTaskGetTickCount();

    //
    // Figure out how long it has been since the last time we've read the
    // continuous timer. We should be reading often enough that we'll never
    // have more than one overflow.
    //
    ui32ElapsedTime = ui32CurrentTime - g_ui32LastTime;

    //
    // Check to see if any WSF ticks need to happen.
    //
    if ( (ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS) > 0 )
    {
        //
        // Update the WSF timers and save the current time as our "last
        // update".
        //
        WsfTimerUpdate(ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS);

        g_ui32LastTime = ui32CurrentTime;
    }
}

//*****************************************************************************
//
// Set a timer interrupt for the next upcoming scheduler event.
//
//*****************************************************************************
void
set_next_wakeup(void)
{
    bool_t bTimerRunning;
    wsfTimerTicks_t xNextExpiration;

    //
    // Check to see when the next timer expiration should happen.
    //
    xNextExpiration = WsfTimerNextExpiration(&bTimerRunning);

    //
    // If there's a pending WSF timer event, set an interrupt to wake us up in
    // time to service it.
    //
    if ( xNextExpiration )
    {
        configASSERT(pdPASS == xTimerChangePeriod( xWsfTimer,
                pdMS_TO_TICKS(xNextExpiration*CLK_TICKS_PER_WSF_TICKS), 100)) ;
    }
}
#endif

//*****************************************************************************
//
// Initialization for the ExactLE stack.
//
//*****************************************************************************
void
exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
    uint16_t       wsfBufMemLen;
    //
    // Set up timers for the WSF scheduler.
    //
    scheduler_timer_init();
    WsfTimerInit();

    //
    // Initialize a buffer pool for WSF dynamic memory needs.
    //
    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem), (uint8_t *)g_pui32BufMem, WSF_BUF_POOLS,
               g_psPoolDescriptors);

    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_debug_printf("Memory pool is too small by %d\r\n",
                             wsfBufMemLen - sizeof(g_pui32BufMem));
    }

    //
    // Initialize the WSF security service.
    //
    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    //
    // Set up callback functions for the various layers of the ExactLE stack.
    //
    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmScanInit();
    DmConnInit();
    DmConnMasterInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    L2cInit();
    L2cMasterInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmpiInit();
    SmpiScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(AmdtpcHandler);
    AmdtpcHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);

#ifdef BLE_MENU
    g_uartDataReadyHandlerId = WsfOsSetNextHandler(uart_data_ready_handler);
#endif
}

#ifdef BLE_MENU
//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;
    char rxData;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_uart_interrupt_status_get(UART, &ui32Status, true);
    am_hal_uart_interrupt_clear(UART, ui32Status);

    //
    // Service the uart FIFO.
    //
    const am_hal_uart_transfer_t sGetChar =
    {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = (uint8_t *) &rxData,
        .ui32NumBytes = 1,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = 0,
    };

    am_hal_uart_transfer(UART, &sGetChar);

    if ((rxData == '\n') || (rxData == '\r'))
    {
        wsfMsgHdr_t  *pMsg;
        if ( (pMsg = WsfMsgAlloc(0)) != NULL )
        {
            WsfMsgSend(g_uartDataReadyHandlerId, pMsg);
        }
    }
    else
    {
        menuRxData[menuRxDataLen++] = rxData;
    }
}
#endif

//*****************************************************************************
//
// Interrupt handler for BLE
//
//*****************************************************************************
void
am_ble_isr(void)
{

    HciDrvIntService();

    // Signal radio task to run

    WsfTaskSetReady(0, 0);
}

//*****************************************************************************
//
// Perform initial setup for the radio task.
//
//*****************************************************************************
void
RadioTaskSetup(void)
{
    am_util_debug_printf("RadioTask: setup\r\n");

    //
    // Create an event handle for our wake-up events.
    //
    xRadioEventHandle = xEventGroupCreate();

    //
    // Make sure we actually allocated space for the events we need.
    //
    while ( xRadioEventHandle == NULL );

    // Pass event object to WSF scheduler
    wsfOsSetEventObject((void*)xRadioEventHandle);

    NVIC_SetPriority(BLE_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

    //
    // Boot the radio.
    //
    HciDrvRadioBoot(1);
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RadioTask(void *pvParameters)
{
#if WSF_TRACE_ENABLED == TRUE
    //
    // Enable ITM
    //
    am_util_debug_printf("Starting wicentric trace:\n\n");
#endif

    //
    // Initialize the main ExactLE stack.
    //
    exactle_stack_init();

    //
    // Start the "Amdtp" profile.
    //
    AmdtpcStart();

    while (1)
    {
        //
        // Calculate the elapsed time from our free-running timer, and update
        // the software timers in the WSF scheduler.
        //
        update_scheduler_timers();
        wsfOsDispatcher();

        //
        // Enable an interrupt to wake us up next time we have a scheduled
        // event.
        //
        set_next_wakeup();

        //
        // Check to see if the WSF routines are ready to go to sleep.
        //
        if ( wsfOsReadyToSleep() )
        {
            //
            // Wait for an event to be posted to the Radio Event Handle.
            //
            xEventGroupWaitBits(xRadioEventHandle, 1, pdTRUE,
                                pdFALSE, portMAX_DELAY);
        }
    }
}
