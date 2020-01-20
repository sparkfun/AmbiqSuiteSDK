//*****************************************************************************
//
//! @file am_devices_mspi_psram.c
//!
//! @brief General Multibit SPI PSRAM driver.
//
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
// This is part of revision 2.3.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_devices_mspi_psram.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_TIMEOUT             1000000

am_hal_mspi_dev_config_t        g_psMSPISettings;

void                            *g_pMSPIHandle;

volatile bool                   g_MSPIDMAComplete;

am_hal_mspi_pio_transfer_t      g_PIOTransaction;
uint32_t                        g_PIOBuffer[32];

volatile uint32_t               g_MSPIInterruptStatus;

bool                            g_MSPISave = false;

am_hal_mspi_dev_config_t  SerialCE0MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(APS6404L)
    .ui8TurnAround        = 8,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  SerialCE1MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(APS6404L)
    .ui8TurnAround        = 8,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  QuadCE0MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(APS6404L)
    .ui8TurnAround        = 6,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
    .bSeparateIO          = false,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  QuadCE1MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(APS6404L)
    .ui8TurnAround        = 6,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE1,
    .bSeparateIO          = false,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

uint32_t g_MSPI_MaxTransSize;

const struct
{
    const uint32_t MHz;
    const uint32_t MaxSize;
} g_Mspi_SpeedMax[] =
{
    {AM_HAL_MSPI_CLK_48MHZ, AM_DEVICES_MSPI_PSRAM_48MHZ_MAX_BYTES},
    {AM_HAL_MSPI_CLK_24MHZ, AM_DEVICES_MSPI_PSRAM_24MHZ_MAX_BYTES},
    {AM_HAL_MSPI_CLK_16MHZ, AM_DEVICES_MSPI_PSRAM_16MHZ_MAX_BYTES},
    {AM_HAL_MSPI_CLK_12MHZ, AM_DEVICES_MSPI_PSRAM_12MHZ_MAX_BYTES},
    {AM_HAL_MSPI_CLK_8MHZ,  AM_DEVICES_MSPI_PSRAM_8MHZ_MAX_BYTES}  // Leave this in for PSRAM initialization at 8MHz.
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

//*****************************************************************************
//
//  MSPI Interrupt Service Routine.
//
//*****************************************************************************
void
am_mspi0_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_pMSPIHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pMSPIHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pMSPIHandle, ui32Status);

    g_MSPIInterruptStatus &= ~ui32Status;
}

#if defined(AM_PART_APOLLO3P)
//*****************************************************************************
//
//  MSPI Interrupt Service Routine.
//
//*****************************************************************************
void
am_mspi1_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_pMSPIHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pMSPIHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pMSPIHandle, ui32Status);

    g_MSPIInterruptStatus &= ~ui32Status;
}


//*****************************************************************************
//
//  MSPI Interrupt Service Routine.
//
//*****************************************************************************
void
am_mspi2_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_pMSPIHandle, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_pMSPIHandle, ui32Status);

    am_hal_mspi_interrupt_service(g_pMSPIHandle, ui32Status);

    g_MSPIInterruptStatus &= ~ui32Status;
}

#endif

void pfnMSPI_PSRAM_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    g_MSPIDMAComplete = true;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
uint32_t am_device_command_write(uint8_t ui8Instr, bool bSendAddr,
                                 uint32_t ui32Addr, uint32_t *pData,
                                 uint32_t ui32NumBytes)
{
    // Create the individual write transaction.
    g_PIOTransaction.ui32NumBytes       = ui32NumBytes;
    g_PIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    g_PIOTransaction.bSendAddr          = bSendAddr;
    g_PIOTransaction.ui32DeviceAddr     = ui32Addr;
    g_PIOTransaction.bSendInstr         = true;
    g_PIOTransaction.ui16DeviceInstr    = ui8Instr;
    g_PIOTransaction.bTurnaround        = false;
    g_PIOTransaction.bQuadCmd           = false;
    g_PIOTransaction.pui32Buffer        = pData;

    // Execute the transction over MSPI.
    return am_hal_mspi_blocking_transfer(g_pMSPIHandle, &g_PIOTransaction,
                                         AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
uint32_t am_device_command_read(uint8_t ui8Instr, bool bSendAddr,
                                uint32_t ui32Addr, uint32_t *pData,
                                uint32_t ui32NumBytes)
{
    // Create the individual write transaction.
    g_PIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    g_PIOTransaction.bSendAddr          = bSendAddr;
    g_PIOTransaction.ui32DeviceAddr     = ui32Addr;
    g_PIOTransaction.bSendInstr         = true;
    g_PIOTransaction.ui16DeviceInstr    = ui8Instr;
    g_PIOTransaction.bTurnaround        = false;
    g_PIOTransaction.ui32NumBytes       = ui32NumBytes;
    g_PIOTransaction.bQuadCmd           = false;
    g_PIOTransaction.pui32Buffer        = pData;

    // Execute the transction over MSPI.
    return am_hal_mspi_blocking_transfer(g_pMSPIHandle, &g_PIOTransaction,
                                      AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Reset the external psram
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_reset(void)
{
    //
    // Send the command sequence to reset the device and return status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_device_command_write(AM_DEVICES_MSPI_PSRAM_RESET_ENABLE, false, 0, g_PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_device_command_write(AM_DEVICES_MSPI_PSRAM_RESET_MEMORY, false, 0, g_PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_id(void)
{
    uint32_t      ui32Status;
    uint32_t      ui32DeviceID = 0;

    //
    // Send the command sequence to read the Device ID and return status.
    //
    ui32Status = am_device_command_read(AM_DEVICES_MSPI_PSRAM_READ_ID, true, 0, &ui32DeviceID, 2);
    am_util_stdio_printf("PSRAM ID is 0x%x\n", ui32DeviceID);
    if ((AM_DEVICES_MSPI_PSRAM_KGD_PASS == ui32DeviceID) &&
       (AM_HAL_STATUS_SUCCESS == ui32Status))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
}

//*****************************************************************************
//
// APS6404L Support
//
//*****************************************************************************
#if defined (APS6404L)
//
// Device specific initialization function.
//
uint32_t am_device_init_psram(am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    ui32Status = am_devices_mspi_psram_id();

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }
//    if (*((uint32_t *)0x50020000) == 0x48EAAD88)
//    {
//        while(1);
//    }
//    while(1);
    //
    // Configure the APS6404L Device mode.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            // Device does not support these modes.
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_device_command_write(AM_DEVICES_MSPI_PSRAM_QUAD_MODE_ENTER, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
uint32_t am_device_deinit_psram(am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t ui32Status;

    //
    // Configure the APS6404L Device mode.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            // Device does not support these modes.
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_device_command_write(AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
//! @brief Initialize the mspi_psram driver.
//!
//! @param psMSPISettings - MSPI device structure describing the target spi psram.
//! @param pHandle - MSPI handler which needs to be return
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_init(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings, void **pHandle)
{
    uint32_t      ui32Status;
    am_hal_mspi_dev_config_t        *psMSPISettings1;
    am_hal_mspi_dev_config_t        *psMSPISettings0;

    //
    // Enable fault detection.
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    g_MSPI_MaxTransSize = 0;
    for (uint32_t i = 0; i < (sizeof(g_Mspi_SpeedMax) / sizeof(g_Mspi_SpeedMax[0])); i++)
    {
        if (g_Mspi_SpeedMax[i].MHz == psMSPISettings->eClockFreq)
        {
            g_MSPI_MaxTransSize = g_Mspi_SpeedMax[i].MaxSize;
            break;
        }
    }
    if ( 0 == g_MSPI_MaxTransSize ) // Return an error if Max Transaction size not found.
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the MSPI for Serial or Quad operation during initialization.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
            psMSPISettings0 = &QuadCE0MSPIConfig;
            psMSPISettings1 = &SerialCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            psMSPISettings0 = &QuadCE1MSPIConfig;
            psMSPISettings1 = &SerialCE1MSPIConfig;
            break;
        default:
            am_util_stdio_printf("Error - Incorrect MSPI Device configuration.\n");
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }

    // First configure in Quad mode and reset
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &g_pMSPIHandle))
    {
        am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_WAKE, g_MSPISave == true ? true : false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if 1
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(g_pMSPIHandle, psMSPISettings0))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(g_pMSPIHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings0->eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_reset())
    {
        am_util_stdio_printf("Error - Failed to reset device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    // Revert to Serial mode
    if (AM_HAL_STATUS_SUCCESS != am_device_deinit_psram(psMSPISettings0))
    {
        am_util_stdio_printf("Error - Failed to return device to serial mode.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error - Failed to disable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(g_pMSPIHandle, psMSPISettings1))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(g_pMSPIHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings1->eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_reset())
    {
        am_util_stdio_printf("Error - Failed to reset device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Initialize the MSPI settings for the MSPI_PSRAM.
    //
    g_psMSPISettings = *psMSPISettings;

    //
    // Device specific MSPI psram initialization.
    //
    ui32Status = am_device_init_psram(psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Disable MSPI before re-configuring it
    ui32Status = am_hal_mspi_disable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(g_pMSPIHandle, psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings->eDeviceConfig);

    //
    // Enable MSPI interrupts.
    //

#if MSPI_USE_CQ

    ui32Status = am_hal_mspi_interrupt_clear(g_pMSPIHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(g_pMSPIHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#else
    ui32Status = am_hal_mspi_interrupt_clear(g_pMSPIHandle, AM_HAL_MSPI_INT_ERR | AM_HAL_MSPI_INT_DMACMP | AM_HAL_MSPI_INT_CMDCMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(g_pMSPIHandle, AM_HAL_MSPI_INT_ERR | AM_HAL_MSPI_INT_DMACMP | AM_HAL_MSPI_INT_CMDCMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

#if AM_CMSIS_REGS
    NVIC_EnableIRQ(mspi_interrupts[ui32Module]);
#else // AM_CMSIS_REGS
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_MSPI);
#endif // AM_CMSIS_REGS

    am_hal_interrupt_master_enable();

    //
    // Return the handle.
    //
    *pHandle = g_pMSPIHandle;

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief DeInitialize the mspi_psram driver.
//!
//! @param psMSPISettings - MSPI device structure describing the target spi psram.
//! @param pHandle - MSPI handler.
//!
//! @return status.
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_deinit(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status;

    //
    // Device specific MSPI psram initialization.
    //
    ui32Status = am_device_deinit_psram(psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Disable the master interrupt on NVIC
    //
    am_hal_interrupt_master_disable();

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(g_pMSPIHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(g_pMSPIHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Disable the MSPI interrupts.
    //
#if AM_CMSIS_REGS
    NVIC_DisableIRQ(mspi_interrupts[ui32Module]);
#else // AM_CMSIS_REGS
    am_hal_interrupt_disable(AM_HAL_INTERRUPT_MSPI);
#endif // AM_CMSIS_REGS

    //
    // Disable MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Disable power to the MSPI instance.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    g_MSPISave = true;
    //
    // Deinitialize the MPSI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

// This function takes care of splitting the transaction as needed, if the transaction crosses
// PSRAM page boundary
static uint32_t psram_nonblocking_transfer(bool bHiPrio, bool bWrite,
                                           uint8_t *pui8Buffer,
                                           uint32_t ui32Address,
                                           uint32_t ui32NumBytes,
                                           uint32_t ui32PauseCondition,
                                           uint32_t ui32StatusSetClr,
                                           am_hal_mspi_callback_t pfnCallback,
                                           void *pCallbackCtxt)
{
    uint32_t ui32Status = AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
    am_hal_mspi_dma_transfer_t    Transaction;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the address configuration for the read
    Transaction.eAddrCfg = AM_HAL_MSPI_ADDR_3_BYTE;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = bWrite ? AM_HAL_MSPI_TX: AM_HAL_MSPI_RX;


    // Initialize the CQ stimulus.
    Transaction.ui32PauseCondition = ui32PauseCondition;
    // Initialize the post-processing
    Transaction.ui32StatusSetClr = 0;

    // Need to be aware of page size
    while (ui32NumBytes)
    {
        uint32_t maxSize = AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - 1));
        uint32_t limit = (maxSize > g_MSPI_MaxTransSize) ? g_MSPI_MaxTransSize : maxSize;
        uint32_t size = (ui32NumBytes > limit) ? limit : ui32NumBytes;
        bool bLast = (size == ui32NumBytes);

        // Set the transfer count in bytes.
        Transaction.ui32TransferCount = size;

        // Set the address to read data from.
        Transaction.ui32DeviceAddress = ui32Address;

        // Set the target SRAM buffer address.
        Transaction.ui32SRAMAddress = (uint32_t)pui8Buffer;

        if (bLast)
        {
            Transaction.ui32StatusSetClr = ui32StatusSetClr;
        }
        if (bHiPrio)
        {
            ui32Status = am_hal_mspi_highprio_transfer(g_pMSPIHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
                                                                 bLast ? pfnCallback : NULL,
                                                                 bLast ? pCallbackCtxt : NULL);
        }
        else
        {
            ui32Status = am_hal_mspi_nonblocking_transfer(g_pMSPIHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
                                                                 bLast ? pfnCallback : NULL,
                                                                 bLast ? pCallbackCtxt : NULL);
        }
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            break;
        }
        ui32Address += size;
        ui32NumBytes -= size;
        pui8Buffer += size;

        Transaction.ui32PauseCondition = 0;
    }
    return ui32Status;
}


//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the psram
//! @param ui32ReadAddress - Address of desired data in external psram
//! @param ui32NumBytes - Number of bytes to read from external psram
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read_adv(uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    ui32Status = psram_nonblocking_transfer(false, false,
                           pui8RxBuffer,
                           ui32ReadAddress,
                           ui32NumBytes,
                           ui32PauseCondition,
                           ui32StatusSetClr,
                           pfnCallback,
                           pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Writes the contents of the external psram into a buffer.
//!
//! @param pui8TxBuffer - Buffer to store the transfer data to the psram
//! @param ui32WriteAddress - Address of desired data in external psram
//! @param ui32NumBytes - Number of bytes to read from external psram
//!
//! This function Writes the external psram at the provided address.
//! This function will only transfer ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write_adv(uint8_t *pui8TxBuffer,
                                uint32_t ui32WriteAddress,
                                uint32_t ui32NumBytes,
                                uint32_t ui32PauseCondition,
                                uint32_t ui32StatusSetClr,
                                am_hal_mspi_callback_t pfnCallback,
                                void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    ui32Status = psram_nonblocking_transfer(false, true,
                                            pui8TxBuffer,
                                            ui32WriteAddress,
                                            ui32NumBytes,
                                            ui32PauseCondition,
                                            ui32StatusSetClr,
                                            pfnCallback,
                                            pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
       return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the psram
//! @param ui32ReadAddress - Address of desired data in external psram
//! @param ui32NumBytes - Number of bytes to read from external psram
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read(uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion)
{
    uint32_t                      ui32Status;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        g_MSPIDMAComplete = false;
        ui32Status = psram_nonblocking_transfer(false, false,
                               pui8RxBuffer,
                               ui32ReadAddress,
                               ui32NumBytes,
                               0,
                               0,
                               pfnMSPI_PSRAM_Callback,
                               NULL);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
        {
            if (g_MSPIDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
        }

        // Check the status.
        if (!g_MSPIDMAComplete)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = psram_nonblocking_transfer(false, false,
                               pui8RxBuffer,
                               ui32ReadAddress,
                               ui32NumBytes,
                               0,
                               0,
                               pfnMSPI_PSRAM_Callback,
                               NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the psram
//! @param ui32ReadAddress - Address of desired data in external psram
//! @param ui32NumBytes - Number of bytes to read from external psram
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read_hiprio(uint8_t *pui8RxBuffer,
                                  uint32_t ui32ReadAddress,
                                  uint32_t ui32NumBytes,
                                  am_hal_mspi_callback_t pfnCallback,
                                  void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    ui32Status = psram_nonblocking_transfer(true, false,
                           pui8RxBuffer,
                           ui32ReadAddress,
                           ui32NumBytes,
                           0,
                           0,
                           pfnCallback,
                           pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external psram at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target psram
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write(uint8_t *pui8TxBuffer,
                            uint32_t ui32WriteAddress,
                            uint32_t ui32NumBytes,
                            bool bWaitForCompletion)
{
    uint32_t                      ui32Status;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        g_MSPIDMAComplete = false;
        ui32Status = psram_nonblocking_transfer(false, true,
                               pui8TxBuffer,
                               ui32WriteAddress,
                               ui32NumBytes,
                               0,
                               0,
                               pfnMSPI_PSRAM_Callback,
                               NULL);

       // Check the transaction status.
       if (AM_HAL_STATUS_SUCCESS != ui32Status)
       {
           return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
       }

       // Wait for DMA Complete or Timeout
       for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
       {
           if (g_MSPIDMAComplete)
           {
               break;
           }
           //
           // Call the BOOTROM cycle function to delay for about 1 microsecond.
           //
           am_hal_flash_delay( FLASH_CYCLES_US(1) );
       }

       // Check the status.
       if (!g_MSPIDMAComplete)
       {
           return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
       }
    }
    else
    {
        // Check the transaction status.
        ui32Status = psram_nonblocking_transfer(false, true,
                               pui8TxBuffer,
                               ui32WriteAddress,
                               ui32NumBytes,
                               0,
                               0,
                               NULL,
                               NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

uint32_t
am_devices_mspi_psram_nonblocking_write(uint8_t *pui8TxBuffer,
                                        uint32_t ui32WriteAddress,
                                        uint32_t ui32NumBytes,
                                        am_hal_mspi_callback_t pfnCallback,
                                        void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(false, true,
                           pui8TxBuffer,
                           ui32WriteAddress,
                           ui32NumBytes,
                           0,
                           0,
                           pfnCallback,
                           pCallbackCtxt);
     if (AM_HAL_STATUS_SUCCESS != ui32Status)
     {
         return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
     }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

uint32_t
am_devices_mspi_psram_nonblocking_read(uint8_t *pui8RxBuffer,
                                       uint32_t ui32ReadAddress,
                                       uint32_t ui32NumBytes,
                                       am_hal_mspi_callback_t pfnCallback,
                                       void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(false, false,
                           pui8RxBuffer,
                           ui32ReadAddress,
                           ui32NumBytes,
                           0,
                           0,
                           pfnCallback,
                           pCallbackCtxt);
     if (AM_HAL_STATUS_SUCCESS != ui32Status)
     {
         return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
     }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external psram at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target psram
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_write_hiprio(uint8_t *pui8TxBuffer,
                                   uint32_t ui32WriteAddress,
                                   uint32_t ui32NumBytes,
                                   am_hal_mspi_callback_t pfnCallback,
                                   void *pCallbackCtxt)
{
    uint32_t                      ui32Status;

    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(true, true,
                           pui8TxBuffer,
                           ui32WriteAddress,
                           ui32NumBytes,
                           0,
                           0,
                           pfnCallback,
                           pCallbackCtxt);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into XIP mode.
//!
//! This function sets the external psram device and the MSPI into XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_enable_xip(void)
{
    uint32_t ui32Status;

    //
    // Enable XIP on the MSPI.
    //
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

#if !MSPI_USE_CQ
    // Disable the DMA interrupts.
    ui32Status = am_hal_mspi_interrupt_disable(g_pMSPIHandle,
                                               AM_HAL_MSPI_INT_DMAERR |
                                               AM_HAL_MSPI_INT_DMACMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from XIP mode.
//!
//! This function removes the external device and the MSPI from XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_disable_xip(void)
{
    uint32_t ui32Status;

    //
    // Disable XIP on the MSPI.
    //
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into scrambling mode.
//!
//! This function sets the external psram device and the MSPI into scrambling mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_enable_scrambling(void)
{
    uint32_t ui32Status;

    //
    // Enable scrambling on the MSPI.
    //
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from scrambling mode.
//!
//! This function removes the external device and the MSPI from scrambling mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_disable_scrambling(void)
{
    uint32_t ui32Status;

    //
    // Disable Scrambling on the MSPI.
    //
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

