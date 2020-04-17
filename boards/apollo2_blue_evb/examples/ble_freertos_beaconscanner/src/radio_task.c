//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief Task to handle radio operation.
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

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "ble_freertos_beaconscanner.h"

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

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"

#include "hci_apollo_config.h"

#include "wsf_msg.h"

//*****************************************************************************
//
// Includes for the ExactLE "fit" profile.
//
//*****************************************************************************
#include "beaconscanner_api.h"
#include "app_ui.h"

//*****************************************************************************
//
// Radio task handle.
//
//*****************************************************************************
TaskHandle_t radio_task_handle;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void exactle_stack_init(void);
void button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

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

wsfHandlerId_t g_bleDataReadyHandlerId;
/*! Event types for ble rx data handler */
#define BLE_DATA_READY_EVENT                   0x01      /*! Trigger Rx data path */

wsfHandlerId_t ButtonHandlerId;
wsfTimer_t ButtonTimer;


void radio_timer_handler(void);

void ble_data_ready_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (FALSE == HciDataReadyISR())
    {
        // trigger event again to handle pending data from BLE controller
        WsfSetEvent(g_bleDataReadyHandlerId, BLE_DATA_READY_EVENT);
    }
}


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
    WsfOsInit();
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
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(BeaconScannerHandler);
    BeaconScannerHandlerInit(handlerId);

    g_bleDataReadyHandlerId = WsfOsSetNextHandler(ble_data_ready_handler);

    ButtonHandlerId = WsfOsSetNextHandler(button_handler);
}


//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = AM_REGn(UART, 0, MIS);
    AM_REGn(UART, 0, IEC) = ui32Status;

    //
    // Allow the HCI driver to respond to the interrupt.
    //
    //HciDrvUartISR(ui32Status);

    // Signal radio task to run

    WsfTaskSetReady(0, 0);

}

//*****************************************************************************
//
// Interrupt handler for the CTS pin
//
//*****************************************************************************
void
radio_cts_handler(void)
{
    // Signal radio task to run

    WsfTaskSetReady(0, 0);

}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);

    //
    // Check to see if this was a wakeup event from the BLE radio.
    //
    if ( ui64Status & AM_HAL_GPIO_BIT(AM_BSP_GPIO_EM9304_INT) )
    {
        WsfSetEvent(g_bleDataReadyHandlerId, BLE_DATA_READY_EVENT);
    }

    //
    // Call the individual pin interrupt handlers for any pin that triggered an
    // interrupt.
    //
    am_hal_gpio_int_service(ui64Status);
}

//*****************************************************************************
//
// Poll the buttons.
//
//*****************************************************************************
void
button_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    //
    // Restart the button timer.
    //
    WsfTimerStartMs(&ButtonTimer, 10);

    //
    // Every time we get a button timer tick, check all of our buttons.
    //
    am_devices_button_array_tick(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // If we got a a press, do something with it.
    //
    if ( am_devices_button_released(am_bsp_psButtons[0]) )
    {
        am_util_debug_printf("Got Button 0 Press\n");
        AppUiBtnTest(APP_UI_BTN_1_SHORT);
    }

    if ( am_devices_button_released(am_bsp_psButtons[1]) )
    {
        am_util_debug_printf("Got Button 1 Press\n");
        AppUiBtnTest(APP_UI_BTN_1_SHORT);
    }

    if ( am_devices_button_released(am_bsp_psButtons[2]) )
    {
        am_util_debug_printf("Got Button 2 Press\n");
    }
}

//*****************************************************************************
//
// Sets up a button interface.
//
//*****************************************************************************
void
setup_buttons(void)
{
    //
    // Enable the buttons for user interaction.
    //
    am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // Start a timer.
    //
    ButtonTimer.handlerId = ButtonHandlerId;
    WsfTimerStartSec(&ButtonTimer, 2);
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
    // Boot the radio.
    //
    HciDrvRadioBoot(0);
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
RadioTask(void *pvParameters)
{
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_GPIO, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    //
    // Register an interrupt handler for BLE event
    //
    am_hal_gpio_int_register(AM_BSP_GPIO_EM9304_INT, radio_cts_handler);

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
    // Enable BLE data ready interrupt
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);

    //
    // Prep the buttons for use
    //
    setup_buttons();

    //
    // Start the Beacon Scanner profile.
    //
    BeaconScannerStart();

    while (1)
    {
        //
        // Calculate the elapsed time from our free-running timer, and update
        // the software timers in the WSF scheduler.
        //
        wsfOsDispatcher();

    }
}
