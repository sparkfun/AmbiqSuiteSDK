//*****************************************************************************
//
//! @file main_task.c
//!
//! @brief Task to handle main operation.
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
#include "freertos_mspi_mspi_display.h"

// Position independent prime function
#define SZ_PRIME_MPI    98
const unsigned char Kc_PRIME_MPI[SZ_PRIME_MPI] =
{
0x70, 0xB4, 0x04, 0x46,  0x00, 0x20, 0x89, 0x1C,  0x8C, 0x42, 0x28, 0xDB,  0x45, 0x1C, 0x02, 0x26,
0x8E, 0x42, 0x20, 0xDA,  0x91, 0xFB, 0xF6, 0xF3,  0x06, 0xFB, 0x13, 0x13,  0xD3, 0xB1, 0x76, 0x1C,
0x8E, 0x42, 0x18, 0xDA,  0x91, 0xFB, 0xF6, 0xF3,  0x06, 0xFB, 0x13, 0x13,  0x93, 0xB1, 0x76, 0x1C,
0x8E, 0x42, 0x10, 0xDA,  0x91, 0xFB, 0xF6, 0xF3,  0x06, 0xFB, 0x13, 0x13,  0x53, 0xB1, 0x76, 0x1C,
0x8E, 0x42, 0x08, 0xDA,  0x91, 0xFB, 0xF6, 0xF3,  0x06, 0xFB, 0x13, 0x13,  0x00, 0x2B, 0x18, 0xBF,
0x76, 0x1C, 0xDD, 0xD1,  0x05, 0x46, 0x51, 0x18,  0x8C, 0x42, 0x28, 0x46,  0xD6, 0xDA, 0x70, 0xBC,
0x70, 0x47,
};



//*****************************************************************************
//
// Composition task handle.
//
//*****************************************************************************
TaskHandle_t main_task_handle;

//*****************************************************************************
//
// Handle for Compose-related events.
//
//*****************************************************************************
EventGroupHandle_t xMainEventHandle;


// MSPI HAL can be accessed both from Render and Compose. So we need to use a mutex to ensure HAL is not re-entered
// This is technically only needed if the RTOS is configured for pre-emptive scheduling
SemaphoreHandle_t xMSPIMutex;


//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_isr(void)
{
    //
    // Read and clear the GPIO interrupt status.
    //
#if defined(AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);

    am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
    am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
    am_hal_gpio_interrupt_service(pGpioIntStatusMask);
#elif defined(AM_PART_APOLLO3)
    uint64_t ui64Status;

    am_hal_gpio_interrupt_status_get(false, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);
    am_hal_gpio_interrupt_service(ui64Status);
#else
    #error Unknown device.
#endif
}

//*****************************************************************************
//
// Perform initial setup for the main task.
//
//*****************************************************************************
void
MainTaskSetup(void)
{
    int iRet;

    am_util_debug_printf("MainTask: setup\r\n");
    //
    // Create an event handle for our wake-up events.
    //
    xMainEventHandle = xEventGroupCreate();

    //
    // Make sure we actually allocated space for the events we need.
    //
    while (xMainEventHandle == NULL);

    NVIC_SetPriority(DISPLAY_MSPI_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(PSRAM_MSPI_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(GPIO_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

    // Initialize the MSPI PSRAM
    iRet = psram_init();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize MSPI psram\n");
        while(1);
    }

    // Initialize the IOM Display

    iRet = display_init();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize Display\n");
        while(1);
    }

    am_hal_interrupt_master_enable();

    xMSPIMutex = xSemaphoreCreateMutex();
    if (xMSPIMutex == NULL)
    {
        DEBUG_PRINT("Unable to Create xMspiMutex\n");
        while(1);
    }
}

typedef enum
{
    DISPLAY_IDLE,
    DISPLAY_SCHEDULED,
    DISPLAY_STARTED
} display_state_e;

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
MainTask(void *pvParameters)
{
    uint32_t eventMask;
    display_state_e displayState = DISPLAY_IDLE;
    uint32_t numFBReadyForDisplay = 0;
    // One FB composition is already started outside
    uint32_t numFBAvailForComposition = NUM_FB - 1;
    bool     bCompositionInProgress = true;

    while (1)
    {
        //
        // Wait for an event to be posted to the Radio Event Handle.
        //
        eventMask = xEventGroupWaitBits(xMainEventHandle, 0xF, pdTRUE,
                            pdFALSE, portMAX_DELAY);
        if (eventMask != 0)
        {
            if (eventMask & MAIN_EVENT_NEW_FRAME_READY)
            {
                numFBReadyForDisplay++;
                bCompositionInProgress = false;
            }
            if (eventMask & MAIN_EVENT_DISPLAY_DONE)
            {
                displayState = DISPLAY_IDLE;
#ifdef SERIALIZE_COMPOSITION_WITH_RENDERING
                // Frame is available for composition only once display rendering is done
                numFBAvailForComposition++;
#endif
            }
#if !defined(SERIALIZE_COMPOSITION_WITH_RENDERING) && defined(START_MSPI_IOM_XFER_ASAP)
            // Frame is available for composition as soon as Rendering starts (indicated by TE)
            if ((displayState == DISPLAY_SCHEDULED) && (eventMask & MAIN_EVENT_TE))
            {
                displayState = DISPLAY_STARTED;
                numFBAvailForComposition++;
            }
#endif
#ifndef START_MSPI_IOM_XFER_ASAP
            if ((displayState == DISPLAY_IDLE) && numFBReadyForDisplay && (eventMask & MAIN_EVENT_TE))
#else
            if ((displayState == DISPLAY_IDLE) && numFBReadyForDisplay)
#endif
            {
                numFBReadyForDisplay--;
#if !defined(SERIALIZE_COMPOSITION_WITH_RENDERING) && !defined(START_MSPI_IOM_XFER_ASAP)
                // Frame is available for composition as soon as Rendering starts (now)
                displayState = DISPLAY_STARTED;
                numFBAvailForComposition++;
#else
                // Frame is not available for composition till Rendering starts (on next TE)
                displayState = DISPLAY_SCHEDULED;
#endif
                // Initiate new frame transfer
                xEventGroupSetBits(xRenderEventHandle, RENDER_EVENT_START_NEW_FRAME);
            }
            if (numFBAvailForComposition && !bCompositionInProgress)
            {
                numFBAvailForComposition--;
                bCompositionInProgress = true;
                // Initiate new frame composition
                xEventGroupSetBits(xComposeEventHandle, COMPOSE_EVENT_START_NEW_FRAME);
            }
        }
    }
}
