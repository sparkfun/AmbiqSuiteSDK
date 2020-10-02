//*****************************************************************************
//
//! @file mspi_power_example.c
//!
//! @brief MSPI Power Profiling Example
//!
//! Purpose: This example is designed to provide a convenient way to assess the
//! expected power to be used by the MSPI device during: a) DMA reads, b) DMA writes
//! and c) XIP operation.  The example is designed to be combined with and load the
//! prime number library to act as the executable in external Quad SPI flash.
//!
//! Additional Information:
//!
//! 1. Build the libprime.a with the project in \libs\prime\iar.
//!
//! 2. Generate prime_mpi.bin using \libs\prime\make_lib.bat
//!
//! 3. Copy prime_mpi.bin into \boards\common3\examples\mspi_power_example
//!
//! 4. Build the mspi_power_example binaries
//!
//! 5. Create the binary with mspi_flash_loader + external executable from Step #1.
//!
//!    ./mspi_testcase_binary_combiner.py --loaderbin iar/bin/mspi_power_example.bin --libbin prime_mpi.bin --off 0x0 --p0 1000 --p1 0 --p2 1 --r 168 --outbin mspi_power_executable --loader-address 0x0000C000 --chipType apollo3p
//!
//! This example can work on:
//! Apollo3p_evb + Cygnus
//! Recommend to use 3.3V power supply voltage.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices_mspi_psram_aps6404l.h"
#include "am_util.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define         MSPI_TEST_SECTOR        0
#define         MSPI_TRANSFER_SIZE      65535
#define         MSPI_LOOPS              83
#define         MSPI_TRIGGER_GPIO       0
#define         MSPI_XIP_BASE_ADDRESS   0x04000000
#define         MSPI_XIP_FUNCTION_SIZE  72

#define         ENABLE_LOGGING

#ifdef ENABLE_LOGGING
#define DEBUG_PRINT am_util_stdio_printf
#else
#define DEBUG_PRINT(...)
#endif

#define         MSPI_TEST_MODULE        1

#define TOGGLE_TRIGGER_GPIO()                                           \
{                                                                       \
    am_hal_gpio_state_write(MSPI_TRIGGER_GPIO, AM_HAL_GPIO_OUTPUT_SET); \
    am_hal_flash_delay(FLASH_CYCLES_US(10));                            \
    am_hal_gpio_state_write(MSPI_TRIGGER_GPIO, AM_HAL_GPIO_OUTPUT_CLEAR);\
}

//*****************************************************************************
//
// Global data
//
//*****************************************************************************
uint8_t                 g_SectorTXBuffer[64*1024];      // 64K SRAM TX buffer
uint8_t                 g_SectorRXBuffer[64*1024];      // 64K SRAM RX buffer.
uint8_t                 g_SectorRXBuffer2[1024];        // 64K SRAM RX buffer.
uint32_t                DMATCBBuffer[4096];
void                    *pMSPIDevHandle;                // MSPI device handle.
void                    *pMSPIHandle;                   // MSPI instance handle.

am_devices_mspi_psram_config_t MSPI_PSRAM_QuadCE0MSPIConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_QUAD_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_24MHZ,
    .ui32NBTxnBufLength       = sizeof(DMATCBBuffer) / sizeof(uint32_t),
    .pNBTxnBuf                = DMATCBBuffer,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
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
    am_mspi_isr1(MSPI_TEST_MODULE)
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

    am_hal_mspi_interrupt_status_get(pMSPIHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(pMSPIHandle, ui32Status);

    am_hal_mspi_interrupt_service(pMSPIHandle, ui32Status);
}

//*****************************************************************************
//
// Private types.
//
//*****************************************************************************
typedef uint32_t (*mspi_xip_test_function_t)(uint32_t, uint32_t, uint32_t);

typedef struct
{
    uint32_t binAddr;
    uint32_t funcOffset;
    uint32_t binSize;
    uint32_t param0;
    uint32_t param1;
    uint32_t param2;
    uint32_t result;
} mspi_xip_test_funcinfo_t;

//*****************************************************************************
//
// Static function to be executed from external flash device
//
//*****************************************************************************
#if defined(__GNUC_STDC_INLINE__)
__attribute__((naked))
static void xip_test_function(void)
{
    __asm
    (
        "   nop\n"              // Just execute NOPs and return.
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   nop\n"
        "   bx      lr\n"
    );
}

#elif defined(__ARMCC_VERSION)
__asm static void xip_test_function(void)
{
    nop                         // Just execute NOPs and return.
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    bx      lr
}

#elif defined(__IAR_SYSTEMS_ICC__)
__stackless static void xip_test_function(void)
{
    __asm("    nop");           // Just execute NOPs and return.
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    nop");
    __asm("    bx      lr");
}
#endif

// Patchable section of binary
extern uint32_t __Patchable[];

// Configurable option to load a position independent library instead
void get_test_function(mspi_xip_test_funcinfo_t *pFuncInfo)
{
    // some of the parameters are controllable through binary patching
    if (__Patchable[0])
    {
        pFuncInfo->binAddr = __Patchable[0];
        pFuncInfo->funcOffset = __Patchable[1];
        pFuncInfo->binSize = __Patchable[2];
        pFuncInfo->param0 = __Patchable[3];
        pFuncInfo->param1 = __Patchable[4];
        pFuncInfo->param2 = __Patchable[5];
        pFuncInfo->result = __Patchable[6];
    }
    else
    {
        pFuncInfo->binAddr = ((uint32_t)&xip_test_function) & 0xFFFFFFFE;
        pFuncInfo->funcOffset = 0;
        pFuncInfo->binSize = MSPI_XIP_FUNCTION_SIZE;
        pFuncInfo->param0 = 0xDEADBEEF;
        pFuncInfo->result = 0xDEADBEEF;
    }
}

int
main(void)
{
    uint32_t      ui32Status;
    mspi_xip_test_funcinfo_t  funcInfo;
    uint32_t                  result;


    // Set the clock frequency.
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    // Set the default cache configuration
    //am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    //am_hal_cachectrl_enable();

    // Configure the board for low power operation.
    am_bsp_low_power_init();

#ifdef ENABLE_LOGGING
    // Initialize the Debug output to uart.
    am_bsp_itm_printf_enable();
    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

#endif
    DEBUG_PRINT("MSPI Power Example\n\n");

    // Initialize the Trigger GPIO.
    am_hal_gpio_pinconfig(MSPI_TRIGGER_GPIO, g_AM_HAL_GPIO_OUTPUT);

    // Set up the MSPI configuration.
    ui32Status = am_devices_mspi_psram_init(MSPI_TEST_MODULE, &MSPI_PSRAM_QuadCE0MSPIConfig, &pMSPIDevHandle, &pMSPIHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to initialize PSRAM.  Status = %8.8X\n", ui32Status);
    }
    NVIC_EnableIRQ(mspi_interrupts[MSPI_TEST_MODULE]);

    am_hal_interrupt_master_enable();

    // Generate data into the Sector Buffer
    for (uint32_t i = 0; i < sizeof(g_SectorTXBuffer); i++)
    {
        g_SectorTXBuffer[i] = (i & 0xFF);
    }

    // Toggle the trigger GPIO.
    TOGGLE_TRIGGER_GPIO();

    // Write the data a number of times.
    for (uint32_t write = 0; write < MSPI_LOOPS; write++)
    {
        // Write the TX buffer into the target sector.
        ui32Status = am_devices_mspi_psram_write(pMSPIDevHandle, g_SectorTXBuffer, MSPI_TEST_SECTOR,
                                                 MSPI_TRANSFER_SIZE, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to write to PSRAM.  Status = %8.8X\n", ui32Status);
        }
    }

    // Toggle the trigger GPIO.
    TOGGLE_TRIGGER_GPIO();

    // Read the data for a number of times.
    for (uint32_t read = 0; read < MSPI_LOOPS; read++)
    {
        // Read the data back into the RX buffer.
        ui32Status = am_devices_mspi_psram_read(pMSPIDevHandle, g_SectorRXBuffer, MSPI_TEST_SECTOR,
                                                MSPI_TRANSFER_SIZE, true);
        if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
        {
            DEBUG_PRINT("Failed to read from PSRAM.  Status = %8.8X\n", ui32Status);
        }
    }

    // Toggle the trigger GPIO.
    TOGGLE_TRIGGER_GPIO();

    // Get the external function information.
    get_test_function(&funcInfo);

    // Cast a pointer to the begining of the sector as the test function to call.
    mspi_xip_test_function_t test_function = (mspi_xip_test_function_t)((MSPI_XIP_BASE_ADDRESS + MSPI_TEST_SECTOR + funcInfo.funcOffset) | 0x00000001);

    // Write the TX buffer into the target sector.
    ui32Status = am_devices_mspi_psram_write(pMSPIDevHandle, (uint8_t *)funcInfo.binAddr, MSPI_TEST_SECTOR, funcInfo.binSize, true);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to write to PSRAM.  Status = %8.8X\n", ui32Status);
    }

    // Set up for XIP operation.
    ui32Status = am_devices_mspi_psram_enable_xip(pMSPIDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to enable MSPI XIP mode.  Status = %8.8X\n", ui32Status);
    }

    // Toggle the trigger GPIO.
    TOGGLE_TRIGGER_GPIO();

    // Execute a call to the test function in the sector.
    result = test_function(funcInfo.param0, funcInfo.param1, funcInfo.param2);
    if (result != funcInfo.result)
    {
        DEBUG_PRINT("Result from XIP execution of function was not as expected.\n");
        DEBUG_PRINT("Param0 = %d Param1 = %d Param2 = %d Result = %d\n",
                             funcInfo.param0, funcInfo.param1, funcInfo.param2, funcInfo.result);
    }

    // Toggle the trigger GPIO.
    TOGGLE_TRIGGER_GPIO();

    // Shutdown XIP operation.
    ui32Status = am_devices_mspi_psram_disable_xip(pMSPIDevHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to disable MSPI XIP mode.  Status = %8.8X\n", ui32Status);
    }

    // Shutdown MSPI
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_deinit(pMSPIDevHandle))
    {
        DEBUG_PRINT("Failed to power off MSPI.\n");
    }

    // Loop forever while sleeping.
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

