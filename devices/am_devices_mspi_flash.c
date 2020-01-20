//*****************************************************************************
//
//! @file am_devices_mspi_flash.c
//!
//! @brief General Multibit SPI Flash driver.
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
#include "am_devices_mspi_flash.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_TIMEOUT             1000000
#define AM_DEVICES_MSPI_FLASH_ERASE_TIMEOUT       1000000

am_hal_mspi_dev_config_t        g_psMSPISettings;

void                            *g_pMSPIHandle;

volatile bool                   g_MSPIDMAComplete;

am_hal_mspi_pio_transfer_t      g_PIOTransaction;
uint32_t                        g_PIOBuffer[32];

volatile uint32_t               g_MSPIInterruptStatus;

am_hal_mspi_dev_config_t  SerialCE0MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(MICRON_N25Q256A)
  .ui8TurnAround        = 3,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (CYPRESS_S25FS064S)
#if defined (MSPI_XIPMIXED)
    .ui8TurnAround        = 4,
#else
    .ui8TurnAround        = 3,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (MACRONIX_MX25U12835F)
  .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (ADESTO_ATXP032)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
#endif
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_FLASH_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_FLASH_PAGE_PROGRAM,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  SerialCE1MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
#if defined(MICRON_N25Q256A)
  .ui8TurnAround        = 3,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (CYPRESS_S25FS064S)
#if defined (MSPI_XIPMIXED)
    .ui8TurnAround        = 4,
#else
    .ui8TurnAround        = 3,
#endif
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (MACRONIX_MX25U12835F)
  .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
#elif defined (ADESTO_ATXP032)
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE,
#endif
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_FLASH_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_FLASH_PAGE_PROGRAM,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  QuadPairedSerialMSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 3,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL,
    .bSeparateIO          = true,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_FLASH_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_FLASH_PAGE_PROGRAM,
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
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
// Forward declarations.
//
uint32_t am_device_command_write(uint32_t ui32Module,
                                 uint8_t ui8Instr,
                                 bool bSendAddr,
                                 uint32_t ui32Addr,
                                 uint32_t *pData,
                                 uint32_t ui32NumBytes);
uint32_t am_device_command_read(uint32_t ui32Module,
                                uint8_t ui8Instr,
                                bool bSendAddr,
                                uint32_t ui32Addr,
                                uint32_t *pData,
                                uint32_t ui32NumBytes);

//*****************************************************************************
//
// Micron N25Q256A Support
//
//*****************************************************************************
#if defined (MICRON_N25Q256A)
//
// Device specific configuration function.
//
uint32_t am_device_init_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status;

    //
    // Configure the turnaround waitstates, xip, and wrap based on MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    g_PIOBuffer[0] =
        AM_DEVICES_MSPI_FLASH_DUMMY_CLOCKS(psMSPISettings->ui8TurnAround)   |
                                           AM_DEVICES_MSPI_FLASH_XIP(1)     |
                                           AM_DEVICES_MSPI_FLASH_WRAP(3);

    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_VOL_CFG, false, 0, g_PIOBuffer, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the MSPI_FLASH mode based on the MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_SERIAL_MODE;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_DUAL_MODE;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_QUAD_MODE;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }

    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENHVOL_CFG, false, 0, g_PIOBuffer, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the MSPI_FLASH byte addressing mode.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    switch (psMSPISettings->eAddrCfg)
    {
        case AM_HAL_MSPI_ADDR_1_BYTE:
        case AM_HAL_MSPI_ADDR_2_BYTE:
        case AM_HAL_MSPI_ADDR_3_BYTE:
            // Exit 4-byte mode.
            ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_EXIT_4B, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            break;
        case AM_HAL_MSPI_ADDR_4_BYTE:
            // Exit 4-byte mode.
            ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_ENTER_4B, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            break;
    }

    // Return status.
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;

}

//
// Device specific configuration function.
//
uint32_t am_device_deinit_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Configure the MSPI_FLASH back to serial mode.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_SERIAL_MODE;
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENHVOL_CFG, false, 0, g_PIOBuffer, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    else
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
    }
}

#endif

//*****************************************************************************
//
// Cypress S25FS064S Support
//
//*****************************************************************************
#if defined (CYPRESS_S25FS064S)
//
// Device specific initialization function.
//
uint32_t am_device_init_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;
    uint8_t       ui8Value;

    //
    // Configure the Cypress S25FS064S Configuration Register 1 Volatile based on the MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

#if 0
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            ui8Value = AM_DEVICES_MSPI_FLASH_SERIAL_MODE;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            ui8Value = AM_DEVICES_MSPI_FLASH_DUAL_MODE;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            ui8Value = AM_DEVICES_MSPI_FLASH_QUAD_MODE;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }
#endif

    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_CR1V | AM_DEVICES_MSPI_FLASH_CR1V_VALUE(AM_DEVICES_MSPI_FLASH_SERIAL_MODE);
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG, false, 0, g_PIOBuffer, 4);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the Cypress S25FS064S Configuration Register 3 Volatile based on the MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_CR3V | AM_DEVICES_MSPI_FLASH_CR3V_VALUE;
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG, false, 0, g_PIOBuffer, 4);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the Cypress S25FS064S Configuration Register 4 Volatile based on the MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_CR4V | AM_DEVICES_MSPI_FLASH_CR4V_VALUE;
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG, false, 0, g_PIOBuffer, 4);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the Cypress S25FS064S Configuration Register 2 Volatile based on the MSPI configuration.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the Addressing mode.
    //
    switch (psMSPISettings->eAddrCfg)
    {
        case AM_HAL_MSPI_ADDR_1_BYTE:
        case AM_HAL_MSPI_ADDR_2_BYTE:
        case AM_HAL_MSPI_ADDR_3_BYTE:
            ui8Value = 0;           // Use the Cypress MSPI Flash in 3-byte mode.
            break;
        case AM_HAL_MSPI_ADDR_4_BYTE:
            ui8Value = 0x80;        // Use the Cypress MSPI Flash in 4-byte mode.
            break;
    }

    //
    // Configure the read latency turnaround value.
    //
    switch (psMSPISettings->eXipMixedMode)
    {
        case AM_HAL_MSPI_XIPMIXED_NORMAL:
        case AM_HAL_MSPI_XIPMIXED_D2:
        case AM_HAL_MSPI_XIPMIXED_D4:
            ui8Value |= psMSPISettings->ui8TurnAround;  // Read latency
            break;
        case AM_HAL_MSPI_XIPMIXED_AD2:
            ui8Value |= psMSPISettings->ui8TurnAround - AM_DEVICES_MSPI_FLASH_ADDR_DUAL_EXT_DELAY;  // Read latency + Mode Bits
            break;
        case AM_HAL_MSPI_XIPMIXED_AD4:
            ui8Value |= psMSPISettings->ui8TurnAround - AM_DEVICES_MSPI_FLASH_ADDR_QUAD_EXT_DELAY;  // Read latency + Mode Bits
            break;
    }

    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            ui8Value |= 0x40;
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_CR2V | AM_DEVICES_MSPI_FLASH_CR2V_VALUE(ui8Value);

    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG, false, 0, g_PIOBuffer, 4);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
uint32_t am_device_deinit_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Configure the Cypress S25FS064S Configuration Register 1 Volatile back to serial mode.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    g_PIOBuffer[0] = AM_DEVICES_MSPI_FLASH_CR1V | AM_DEVICES_MSPI_FLASH_CR1V_VALUE(AM_DEVICES_MSPI_FLASH_SERIAL_MODE);
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG, false, 0, g_PIOBuffer, 4);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    else
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
    }
}
#endif

//*****************************************************************************
//
// Macronix MX25U12335F Support
//
//*****************************************************************************
#if defined (MACRONIX_MX25U12835F)
//
// Device specific initialization function.
//
uint32_t am_device_init_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;

#if 0
    //
    // Enable writing to the Status/Configuration register.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the Macronix MX25U12835F Status/Configuration Register.
    //
    g_PIOBuffer[0] = (AM_DEVICES_MSPI_FLASH_CONFIG << 8) | AM_DEVICES_MSPI_FLASH_STATUS;
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_STATUS, false, 0, g_PIOBuffer, 2);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Read the Configuration Register.
    //
    ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_CONFIG, false, 0, g_PIOBuffer, 1);
#endif

    //
    // Configure the Macronix MX25U2835F Device mode.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            // Device does not support Dual mode.
            //return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_ENABLE_QPI_MODE, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
uint32_t am_device_deinit_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status;

    //
    // Configure the Macronix MX25U2835F Device mode.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            // Device does not support Dual mode.
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_DISABLE_QPI_MODE, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}
#endif

am_hal_mspi_dev_config_t am_devices_mspi_flash_mode_switch(uint32_t ui32Module,
                                                           am_hal_mspi_dev_config_t *pMSPISettings)
{
    uint32_t ui32Status = AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
    am_hal_mspi_dev_config_t MSPISettingsBak;
    if ( pMSPISettings )
    {
        MSPISettingsBak = g_psMSPISettings;
        g_psMSPISettings = *pMSPISettings;

        ui32Status = am_hal_mspi_disable(g_pMSPIHandle);

        ui32Status |= am_hal_mspi_device_configure(g_pMSPIHandle, &g_psMSPISettings);

        ui32Status |= am_hal_mspi_enable(g_pMSPIHandle);

        am_bsp_mspi_pins_enable(ui32Module, g_psMSPISettings.eDeviceConfig);

        ui32Status |= am_hal_mspi_interrupt_clear(g_pMSPIHandle, AM_HAL_MSPI_INT_ERR | AM_HAL_MSPI_INT_DMACMP );
        ui32Status |= am_hal_mspi_interrupt_enable(g_pMSPIHandle, AM_HAL_MSPI_INT_ERR | AM_HAL_MSPI_INT_DMACMP );

        NVIC_EnableIRQ(mspi_interrupts[ui32Module]);
    }

    return MSPISettingsBak;
}

void am_devices_mspi_mixmode_switch(uint32_t ui32Mode)
{
    am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_XIP_MIXED_MODE, &ui32Mode);
}

//*****************************************************************************
//
// Adesto ATXP032 Support
//
//*****************************************************************************
#if defined (ADESTO_ATXP032)
//
// Device specific initialization function.
//
uint32_t am_device_init_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{

  //
  // Set the Dummy Cycles in Status/Control register 3 to 8.
  //
  am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
  g_PIOBuffer[0] = 0x00000003;
  am_device_command_write(ui32Module, AM_DEVICES_ATXP032_WRITE_STATUS_CTRL, false, 0, g_PIOBuffer, 2);

  //
  // Configure the ATXP032 mode based on the MSPI configuration.
  //
  am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);

    switch ( psMSPISettings->eDeviceConfig )
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            am_device_command_write(ui32Module, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, g_PIOBuffer, 0);
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            am_device_command_write(ui32Module, AM_DEVICES_ATXP032_ENTER_QUAD_MODE, false, 0, g_PIOBuffer, 0);
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            am_device_command_write(ui32Module, AM_DEVICES_ATXP032_ENTER_OCTAL_MODE, false, 0, g_PIOBuffer, 0);
            break;
    }

  return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
uint32_t am_device_deinit_flash(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status;

    //
    // Configure the Adesto ATXP032 Device mode.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            // Device does not support Dual mode.
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            ui32Status = am_device_command_write(ui32Module, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, g_PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            break;
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
uint32_t am_device_command_write(uint32_t ui32Module, uint8_t ui8Instr, bool bSendAddr,
                                 uint32_t ui32Addr, uint32_t *pData,
                                 uint32_t ui32NumBytes)
{
    uint32_t ui32Status;

    // Create the individual write transaction.
    g_PIOTransaction.ui32NumBytes       = ui32NumBytes;
    g_PIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    g_PIOTransaction.bSendAddr          = bSendAddr;
    g_PIOTransaction.ui32DeviceAddr     = ui32Addr;
    g_PIOTransaction.bSendInstr         = true;
    g_PIOTransaction.ui16DeviceInstr    = ui8Instr;
    g_PIOTransaction.bTurnaround        = false;
#if 0 // A3DS-25 Deprecate MSPI CONT
    g_PIOTransaction.bContinue          = false;
#endif // A3DS-25

    if (AM_HAL_MSPI_FLASH_QUADPAIRED == g_psMSPISettings.eDeviceConfig)
    {
        g_PIOTransaction.bQuadCmd         = true;
    }
    else
    {
        g_PIOTransaction.bQuadCmd         = false;
    }

    g_PIOTransaction.pui32Buffer        = pData;

#if defined (MSPI_XIPMIXED)
    am_hal_mspi_dev_config_t mode = am_devices_mspi_flash_mode_switch(ui32Module, &SerialCE0MSPIConfig);
#endif

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(g_pMSPIHandle, &g_PIOTransaction,
                                         AM_DEVICES_MSPI_FLASH_TIMEOUT);
#if defined (MSPI_XIPMIXED)
    am_devices_mspi_flash_mode_switch(ui32Module, &mode);
#endif

    return ui32Status;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
uint32_t am_device_command_read(uint32_t ui32Module, uint8_t ui8Instr, bool bSendAddr,
                                uint32_t ui32Addr, uint32_t *pData,
                                uint32_t ui32NumBytes)
{
    uint32_t ui32Status;

    // Create the individual write transaction.
    g_PIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    g_PIOTransaction.bSendAddr          = bSendAddr;
    g_PIOTransaction.ui32DeviceAddr     = ui32Addr;
    g_PIOTransaction.bSendInstr         = true;
    g_PIOTransaction.ui16DeviceInstr    = ui8Instr;
    g_PIOTransaction.bTurnaround        = false;
#if 0 // A3DS-25 Deprecate MSPI CONT
    g_PIOTransaction.bContinue          = false;
#endif // A3DS-25

    if (AM_HAL_MSPI_FLASH_QUADPAIRED == g_psMSPISettings.eDeviceConfig)
    {
        g_PIOTransaction.ui32NumBytes     = ui32NumBytes * 2;
        g_PIOTransaction.bQuadCmd      = true;
    }
    else
    {
        g_PIOTransaction.ui32NumBytes     = ui32NumBytes;
        g_PIOTransaction.bQuadCmd      = false;
    }

    g_PIOTransaction.pui32Buffer        = pData;

#if defined (MSPI_XIPMIXED)
    am_hal_mspi_dev_config_t mode = am_devices_mspi_flash_mode_switch(AM_DEVICES_MSPI_FLASH_MSPI_INSTANCE, &SerialCE0MSPIConfig);
#endif

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(g_pMSPIHandle, &g_PIOTransaction,
                                         AM_DEVICES_MSPI_FLASH_TIMEOUT);
#if defined (MSPI_XIPMIXED)
    am_devices_mspi_flash_mode_switch(AM_DEVICES_MSPI_FLASH_MSPI_INSTANCE, &mode);
#endif

    return ui32Status;
}

void pfnMSPI_FLASH_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    g_MSPIDMAComplete = true;
}

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

//*****************************************************************************
//
//! @brief Initialize the mspi_flash driver.
//!
//! @param psIOMSettings - IOM device structure describing the target spiflash.
//! @param pfnWriteFunc - Function to use for spi writes.
//! @param pfnReadFunc - Function to use for spi reads.
//!
//! This function should be called before any other am_devices_spiflash
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//!
//! @return None.
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_init(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings, void **pHandle)
{
    uint32_t      ui32Status;

    //
    // Enable fault detection.
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    //
    // Configure the MSPI for Serial or Quad-Paired Serial operation during initialization.
    //
    switch (psMSPISettings->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
            g_psMSPISettings = SerialCE0MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(g_pMSPIHandle, &SerialCE0MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            am_bsp_mspi_pins_enable(ui32Module, SerialCE0MSPIConfig.eDeviceConfig);
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            g_psMSPISettings = SerialCE1MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(g_pMSPIHandle, &SerialCE1MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            am_bsp_mspi_pins_enable(ui32Module, SerialCE1MSPIConfig.eDeviceConfig);
            break;
        case AM_HAL_MSPI_FLASH_QUADPAIRED:
            g_psMSPISettings = QuadPairedSerialMSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(g_pMSPIHandle, &QuadPairedSerialMSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(g_pMSPIHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }
            am_bsp_mspi_pins_enable(ui32Module, QuadPairedSerialMSPIConfig.eDeviceConfig);
            break;
        case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            //break;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_flash_reset(ui32Module))
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Device specific MSPI Flash initialization.
    //
    ui32Status = am_device_init_flash(ui32Module, psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Initialize the MSPI settings for the MSPI_FLASH.
    //
    g_psMSPISettings = *psMSPISettings;

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(g_pMSPIHandle, psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, g_psMSPISettings.eDeviceConfig);

    //
    // Enable MSPI interrupts.
    //

    ui32Status = am_hal_mspi_interrupt_clear(g_pMSPIHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(g_pMSPIHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }


    NVIC_EnableIRQ(mspi_interrupts[ui32Module]);

    am_hal_interrupt_master_enable();

    //
    // Return the handle.
    //
    *pHandle = g_pMSPIHandle;

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief De-Initialization the mspi_flash driver.
//!
//! @param psIOMSettings - IOM device structure describing the target spiflash.
//! @param pfnWriteFunc - Function to use for spi writes.
//! @param pfnReadFunc - Function to use for spi reads.
//!
//! This function should be called before any other am_devices_spiflash
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//!
//! @return None.
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_deinit(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings)
{
    uint32_t      ui32Status;

    //
    // Device specific MSPI Flash de-initialization.
    //
    ui32Status = am_device_deinit_flash(ui32Module, psMSPISettings);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_flash_reset(ui32Module))
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
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
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(g_pMSPIHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Disable the MSPI interrupts.
    //
    NVIC_DisableIRQ(mspi_interrupts[ui32Module]);

    //
    // Disable the MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(g_pMSPIHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
      am_util_stdio_printf("Error - Failed to power on MSPI.\n");
      return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Deinitialize the MSPI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(g_pMSPIHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Clear the Flash Caching.
    //
#if AM_CMSIS_REGS
    CACHECTRL->CACHECFG = 0;
#else // AM_CMSIS_REGS
    AM_REG(CACHECTRL, CACHECFG) = 0;
#endif // AM_CMSIS_REGS

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the current status of the external flash
//!
//! @param ui32DeviceNumber - Device number of the external flash
//!
//! This function reads the device ID register of the external flash, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_reset(uint32_t ui32Module)
{
#if defined(ADESTO_ATXP032)
 //
  // Return the device to SPI mode.
  //
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(ui32Module, AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE, false, 0, g_PIOBuffer, 0))
  {
    return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
  }
#else
    //
    // Send the command sequence to reset the device and return status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_RESET_ENABLE, false, 0, g_PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_RESET_MEMORY, false, 0, g_PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
#endif

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the ID of the external flash and returns the value.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external flash, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_id(uint32_t ui32Module)
{
    uint32_t      ui32Status;
    uint32_t      ui32DeviceID;

    //
    // Send the command sequence to read the Device ID and return status.
    //
#if defined(ADESTO_ATXP032)
  uint8_t       ui8Response[11];
  ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_ID, false, 0, (uint32_t *)&ui8Response[0], 11);
  ui32DeviceID = (ui8Response[7] << 16) | (ui8Response[8] << 8) | ui8Response[9];
#else
    ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_ID, false, 0, &ui32DeviceID, 3);
    am_util_stdio_printf("Flash ID is %8.8X\n", ui32DeviceID);
#endif
    if ( ((ui32DeviceID & AM_DEVICES_MSPI_FLASH_ID_MASK) == AM_DEVICES_MSPI_FLASH_ID) &&
       (AM_HAL_STATUS_SUCCESS == ui32Status) )
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//! @brief Reads the current status of the external flash
//!
//! This function reads the status register of the external flash, and returns
//! the result as an 8-bit unsigned integer value. The processor will block
//! during the data transfer process, but will return as soon as the status
//! register had been read.
//!
//! Macro definitions for interpreting the contents of the status register are
//! included in the header file.
//!
//! @return 8-bit status register contents
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_status(uint32_t ui32Module, uint32_t *pStatus)
{
    uint32_t      ui32Status;

    //
    // Send the command sequence to read the device status.
    //
    ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, pStatus, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

uint32_t
am_devices_mspi_flash_read_adv(uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = ui32PauseCondition;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = ui32StatusSetClr;

    // Check the transaction status.
    ui32Status = am_hal_mspi_nonblocking_transfer(g_pMSPIHandle, &Transaction,
                                                  AM_HAL_MSPI_TRANS_DMA, pfnCallback, pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_read(uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        g_MSPIDMAComplete = false;
        ui32Status = am_hal_mspi_nonblocking_transfer(g_pMSPIHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_FLASH_Callback, NULL);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_TIMEOUT; i++)
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
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_nonblocking_transfer(g_pMSPIHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_read_hiprio(uint8_t *pui8RxBuffer,
                                  uint32_t ui32ReadAddress,
                                  uint32_t ui32NumBytes,
                                  bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;

    // Set the DMA priority
    Transaction.ui8Priority = 1;

    // Set the transfer direction to RX (Read)
    Transaction.eDirection = AM_HAL_MSPI_RX;

    // Set the transfer count in bytes.
    Transaction.ui32TransferCount = ui32NumBytes;

    // Set the address to read data from.
    Transaction.ui32DeviceAddress = ui32ReadAddress;

    // Set the target SRAM buffer address.
    Transaction.ui32SRAMAddress = (uint32_t)pui8RxBuffer;

    // Clear the CQ stimulus.
    Transaction.ui32PauseCondition = 0;
    // Clear the post-processing
    Transaction.ui32StatusSetClr = 0;

    if (bWaitForCompletion)
    {
        // Start the transaction.
        g_MSPIDMAComplete = false;
        ui32Status = am_hal_mspi_highprio_transfer(g_pMSPIHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_FLASH_Callback, NULL);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_TIMEOUT; i++)
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
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_highprio_transfer(g_pMSPIHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}



//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param ui32DeviceNumber - Device number of the external flash
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_write(uint32_t ui32Module, uint8_t *pui8TxBuffer,
                            uint32_t ui32WriteAddress,
                            uint32_t ui32NumBytes)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    bool                          bWriteComplete = false;
    uint32_t                      ui32BytesLeft = ui32NumBytes;
    uint32_t                      ui32PageAddress = ui32WriteAddress;
    uint32_t                      ui32BufferAddress = (uint32_t)pui8TxBuffer;
    uint32_t                      ui32Status;

    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

        // Set the DMA priority
        Transaction.ui8Priority = 1;

        // Set the transfer direction to TX (Write)
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_FLASH_PAGE_SIZE)
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_FLASH_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_FLASH_PAGE_SIZE;
        }
        else
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = ui32BytesLeft;
            ui32BytesLeft = 0;
        }

        // Set the address configuration for the read
        //    Transaction.eAddrCfg = AM_HAL_MSPI_ADDR_3_BYTE;

        //    Transaction.eAddrExtSize = AM_HAL_MSPI_ADDR_EXT_0_BYTE;
        //    Transaction.ui8AddrExtValue = 0;

        // Set the address to read data to.
        Transaction.ui32DeviceAddress = ui32PageAddress;
        ui32PageAddress += AM_DEVICES_MSPI_FLASH_PAGE_SIZE;

        // Set the source SRAM buffer address.
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_FLASH_PAGE_SIZE;

        // Clear the CQ stimulus.
        Transaction.ui32PauseCondition = 0;
        // Clear the post-processing
        Transaction.ui32StatusSetClr = 0;

        // Start the transaction.
        g_MSPIDMAComplete = false;
        ui32Status = am_hal_mspi_nonblocking_transfer(g_pMSPIHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_FLASH_Callback, NULL);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_TIMEOUT; i++)
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
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

#if defined (ADESTO_ATXP032)
        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_TIMEOUT; i++)
        {
            // ATXP032 has different number of bytes for each speed of status read.
            switch ( g_psMSPISettings.eDeviceConfig )
            {
                case AM_HAL_MSPI_FLASH_SERIAL_CE0:
                case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                    am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 2);
                    bWriteComplete = (0 == (g_PIOBuffer[0] & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_QUAD_CE0:
                case AM_HAL_MSPI_FLASH_QUAD_CE1:
                    am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 4);
                    bWriteComplete = (0 == ((g_PIOBuffer[0] >> 16) & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_OCTAL_CE0:
                case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                    am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 6);
                    bWriteComplete = (0 == (g_PIOBuffer[1] & AM_DEVICES_ATXP032_WIP));
                    break;
                case AM_HAL_MSPI_FLASH_DUAL_CE0:
                case AM_HAL_MSPI_FLASH_DUAL_CE1:
                case AM_HAL_MSPI_FLASH_QUADPAIRED:
                case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
                    return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
            }

            am_util_delay_us(100);
            if (bWriteComplete)
            {
                break;
            }
        }

#else
        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_TIMEOUT; i++)
        {
          ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 1);
          if ((AM_HAL_MSPI_FLASH_QUADPAIRED == g_psMSPISettings.eDeviceConfig) ||
              (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == g_psMSPISettings.eDeviceConfig))
          {
            bWriteComplete = ((0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP)) &&
                              (0 == ((g_PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_FLASH_WIP)));
          }
          else
          {
            bWriteComplete = (0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP));
          }
          am_util_delay_us(100);
          if (bWriteComplete)
          {
            break;
          }
        }
#endif

        //
        // Send the command sequence to disable writing.
        //
        ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_DISABLE, false, 0, g_PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Erases the entire contents of the external flash
//!
//! @param ui32DeviceNumber - Device number of the external flash
//!
//! This function uses the "Bulk Erase" instruction to erase the entire
//! contents of the external flash.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_mass_erase(uint32_t ui32Module)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the mass erase.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_BULK_ERASE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_ERASE_TIMEOUT; i++)
    {
        g_PIOBuffer[0] = 0;
        am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 1);
        if ((AM_HAL_MSPI_FLASH_QUADPAIRED == g_psMSPISettings.eDeviceConfig) ||
            (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == g_psMSPISettings.eDeviceConfig))
        {
            bEraseComplete = ((0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP)) &&
                              (0 == ((g_PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_FLASH_WIP)));
        }
        else
        {
            bEraseComplete = (0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP));
        }
        if (bEraseComplete)
        {
            break;
        }
        am_util_delay_ms(10);
    }

    //
    // Check the status.
    //
    if (!bEraseComplete)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_DISABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Erases the contents of a single sector of flash
//!
//! @param ui32DeviceNumber - Device number of the external flash
//! @param ui32SectorAddress - Address to erase in the external flash
//!
//! This function erases a single sector of the external flash as specified by
//! ui32EraseAddress. The entire sector where ui32EraseAddress will
//! be erased.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_sector_erase(uint32_t ui32Module, uint32_t ui32SectorAddress)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

#if defined (ADESTO_ATXP032)
    //
    // Send the command to remove protection from the sector.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_ATXP032_UNPROTECT_SECTOR, true, ui32SectorAddress, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
#endif

    //
    // Send the command sequence to do the sector erase.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_SECTOR_ERASE, true, ui32SectorAddress, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

#if defined (ADESTO_ATXP032)
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_ERASE_TIMEOUT; i++)
    {
        // ATXP032 has different number of bytes for each speed of status read.
        switch ( g_psMSPISettings.eDeviceConfig )
        {
            case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            case AM_HAL_MSPI_FLASH_SERIAL_CE1:
                am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 2);
                bEraseComplete = (0 == (g_PIOBuffer[0] & AM_DEVICES_ATXP032_WIP));
                break;
            case AM_HAL_MSPI_FLASH_QUAD_CE0:
            case AM_HAL_MSPI_FLASH_QUAD_CE1:
                am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 4);
                bEraseComplete = (0 == ((g_PIOBuffer[0] >> 16) & AM_DEVICES_ATXP032_WIP));
                break;
            case AM_HAL_MSPI_FLASH_OCTAL_CE0:
            case AM_HAL_MSPI_FLASH_OCTAL_CE1:
                am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 6);
                bEraseComplete = (0 == (g_PIOBuffer[1] & AM_DEVICES_ATXP032_WIP));
                break;
            case AM_HAL_MSPI_FLASH_DUAL_CE0:
            case AM_HAL_MSPI_FLASH_DUAL_CE1:
            case AM_HAL_MSPI_FLASH_QUADPAIRED:
            case AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL:
                return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }

        if (bEraseComplete)
        {
            break;
        }
        am_util_delay_ms(10);
    }
#else
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_FLASH_ERASE_TIMEOUT; i++)
    {
        g_PIOBuffer[0] = 0;
        am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS, false, 0, g_PIOBuffer, 1);
        if ((AM_HAL_MSPI_FLASH_QUADPAIRED == g_psMSPISettings.eDeviceConfig) ||
            (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == g_psMSPISettings.eDeviceConfig))
        {
            bEraseComplete = ((0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP)) &&
                              (0 == ((g_PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_FLASH_WIP)));
        }
        else
        {
            bEraseComplete = (0 == (g_PIOBuffer[0] & AM_DEVICES_MSPI_FLASH_WIP));
        }
        if (bEraseComplete)
        {
            break;
        }
        am_util_delay_ms(10);
    }

#endif
    //
    // Check the status.
    //
    if (!bEraseComplete)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

#if defined (CYPRESS_S25FS064S)
    uint32_t    ui32EraseStatus = 0;
    //
    // Send the command sequence to check the erase status.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_EVAL_ERASE_STATUS, true, ui32SectorAddress, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Send the command sequence to read the device status.
    //
    ui32Status = am_device_command_read(ui32Module, AM_DEVICES_MSPI_FLASH_READ_STATUS2, false, 0, &ui32EraseStatus, 1);
    if ( (AM_HAL_STATUS_SUCCESS != ui32Status) || ((AM_DEVICES_MSPI_FLASH_ERASE_SUCCESS | ui32EraseStatus) == 0) )
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
#endif

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_DISABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Sets up the MSPI and external Flash into XIP mode.
//!
//! This function sets the external flash device and the MSPI into XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_enable(bool bEnableXIP, bool bEnableScrambling)
{
    uint32_t      ui32Status;


    //
    // Enable XIP on the MSPI.
    //
    if (bEnableXIP)
    {
        ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }

    //
    // Enable scrambling on the MSPI.
    //
    if (bEnableScrambling)
    {
        ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Removes the MSPI and external Flash from XIP mode.
//!
//! This function removes the external device and the MSPI from XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_flash_disable_xip(uint32_t ui32Module)
{
    uint32_t      ui32Status;

    //
    // Send the command to enable writing.
    //
    ui32Status = am_device_command_write(ui32Module, AM_DEVICES_MSPI_FLASH_WRITE_ENABLE, false, 0, g_PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Disable XIP and Scrambling on the MSPI.
    //
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_control(g_pMSPIHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_FLASH_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS;
}


