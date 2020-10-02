//*****************************************************************************
//
//! @file am_devices_mspi_mx25u12835f.c
//!
//! @brief General Multibit SPI Mx25u12835f driver.
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

#include "am_mcu_apollo.h"
#include "am_devices_mspi_mx25u12835f.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_MX25U12835F_TIMEOUT             1000000
#define AM_DEVICES_MSPI_MX25U12835F_ERASE_TIMEOUT       1000000

typedef struct
{
    uint32_t                    ui32Module;
    void                        *pMspiHandle;
    am_hal_mspi_dev_config_t    stSetting;
    bool                        bOccupied;
} am_devices_mspi_mx25u12835f_t;

am_devices_mspi_mx25u12835f_t gAmMx25u12835f[AM_DEVICES_MSPI_MX25U12835F_MAX_DEVICE_NUM];

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Serial_CE0_MSPIConfig =
{
    .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Serial_CE1_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 8,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Dual_CE0_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 4,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_DUAL_CE0,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_DUAL_IO_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Dual_CE1_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 4,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_DUAL_CE1,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_DUAL_IO_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Quad_CE0_MSPIConfig =
{
    .ui8TurnAround        = 4,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .bSendAddr            = true,
    .bSendInstr           = true,
    .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t MSPI_Mx25u12835f_Quad_CE1_MSPIConfig =
{
    .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
    .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
    .ui8TurnAround        = 4,
    .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
    .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
    .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE1,
    .bSendInstr           = true,
    .bSendAddr            = true,
    .bTurnaround          = true,
    .ui8ReadInstr         = AM_DEVICES_MSPI_MX25U12835F_FAST_READ,
    .ui8WriteInstr        = AM_DEVICES_MSPI_MX25U12835F_PAGE_PROGRAM,
#if defined(AM_PART_APOLLO3P)
    .ui8WriteLatency      = 0,
    .bEnWriteLatency      = false,
    .bEmulateDDR          = false,
    .ui16DMATimeLimit     = 0,
    .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE,
#endif
    .ui32TCBSize          = 0,
    .pTCB                 = NULL,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
};

struct
{
    am_hal_mspi_device_e eHalDeviceEnum;
    am_hal_mspi_dev_config_t *psDevConfig;
}g_MSPIMx25u12835f_DevConfig[] =
{
    {AM_HAL_MSPI_FLASH_SERIAL_CE0,                   &MSPI_Mx25u12835f_Serial_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_SERIAL_CE1,                   &MSPI_Mx25u12835f_Serial_CE1_MSPIConfig},
    {AM_HAL_MSPI_FLASH_DUAL_CE0,                     &MSPI_Mx25u12835f_Dual_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_DUAL_CE1,                     &MSPI_Mx25u12835f_Dual_CE1_MSPIConfig},
    {AM_HAL_MSPI_FLASH_QUAD_CE0,                     &MSPI_Mx25u12835f_Quad_CE0_MSPIConfig},
    {AM_HAL_MSPI_FLASH_QUAD_CE1,                     &MSPI_Mx25u12835f_Quad_CE1_MSPIConfig},
};

//
// Forward declarations.
//
static uint32_t am_devices_mspi_mx25u12835f_command_write(void *pHandle,
                                                          uint8_t ui8Instr,
                                                          bool bSendAddr,
                                                          uint32_t ui32Addr,
                                                          uint32_t *pData,
                                                          uint32_t ui32NumBytes);
static uint32_t am_devices_mspi_mx25u12835f_command_read(void *pHandle,
                                                         uint8_t ui8Instr,
                                                         bool bSendAddr,
                                                         uint32_t ui32Addr,
                                                         uint32_t *pData,
                                                         uint32_t ui32NumBytes);

//*****************************************************************************
//
// Macronix MX25U12335F Support
//
//*****************************************************************************
//
// Device specific initialization function.
//
static uint32_t
am_device_init_flash(void *pHandle)
{
    uint32_t      ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t      ui32PIOBuffer[32] = {0};
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

#if 0
    //
    // Enable writing to the Status/Configuration register.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Configure the Macronix MX25U12835F Status/Configuration Register.
    //
    ui32PIOBuffer[0] = (AM_DEVICES_MSPI_MX25U12835F_CONFIG << 8) | AM_DEVICES_MSPI_MX25U12835F_STATUS;
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_STATUS, false, 0, ui32PIOBuffer, 2);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Read the Configuration Register.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_CONFIG, false, 0, ui32PIOBuffer, 1);
#endif

    //
    // Configure the Macronix MX25U2835F Device mode.
    //
    switch (pFlash->stSetting.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            // Nothing to do.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_ENABLE_QPI_MODE, false, 0, ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//
// Device specific de-initialization function.
//
static uint32_t
am_device_deinit_flash(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Configure the Macronix MX25U2835F Device mode.
    //
    switch (pFlash->stSetting.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
            // Device does not support Dual mode.
            //return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_DISABLE_QPI_MODE, false, 0, ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            //break;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_devices_mspi_mx25u12835f_command_write(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                          uint32_t ui32Addr, uint32_t *pData,
                                          uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    am_hal_mspi_pio_transfer_t      stMSPIFlashPIOTransaction = {0};

    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.ui32NumBytes       = ui32NumBytes;
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_TX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
#if 0 // A3DS-25 Deprecate MSPI CONT
    stMSPIFlashPIOTransaction.bContinue          = false;
#endif // A3DS-25

    stMSPIFlashPIOTransaction.bQuadCmd         = false;

    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_MX25U12835F_TIMEOUT);

    return ui32Status;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_devices_mspi_mx25u12835f_command_read(void *pHandle, uint8_t ui8Instr, bool bSendAddr,
                                         uint32_t ui32Addr, uint32_t *pData,
                                         uint32_t ui32NumBytes)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    am_hal_mspi_pio_transfer_t      stMSPIFlashPIOTransaction = {0};

    // Create the individual write transaction.
    stMSPIFlashPIOTransaction.eDirection         = AM_HAL_MSPI_RX;
    stMSPIFlashPIOTransaction.bSendAddr          = bSendAddr;
    stMSPIFlashPIOTransaction.ui32DeviceAddr     = ui32Addr;
    stMSPIFlashPIOTransaction.bSendInstr         = true;
    stMSPIFlashPIOTransaction.ui16DeviceInstr    = ui8Instr;
    stMSPIFlashPIOTransaction.bTurnaround        = false;
#if 0 // A3DS-25 Deprecate MSPI CONT
    stMSPIFlashPIOTransaction.bContinue          = false;
#endif // A3DS-25

    stMSPIFlashPIOTransaction.ui32NumBytes     = ui32NumBytes;
    stMSPIFlashPIOTransaction.bQuadCmd      = false;

    stMSPIFlashPIOTransaction.pui32Buffer        = pData;

    // Execute the transction over MSPI.
    ui32Status = am_hal_mspi_blocking_transfer(pFlash->pMspiHandle, &stMSPIFlashPIOTransaction,
                                         AM_DEVICES_MSPI_MX25U12835F_TIMEOUT);

    return ui32Status;
}

static void
pfnMSPI_MX25U12835F_Callback(void *pCallbackCtxt, uint32_t status)
{
    // Set the DMA complete flag.
    *(volatile bool *)pCallbackCtxt = true;
}

//*****************************************************************************
//
//! @brief Initialize the mspi_mx25u12835f driver.
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
am_devices_mspi_mx25u12835f_init(uint32_t ui32Module, am_devices_mspi_mx25u12835f_config_t *psMSPISettings, void **ppHandle, void **ppMspiHandle)
{
    uint32_t      ui32Status;
    am_hal_mspi_dev_config_t *psConfig;
    void                     *pMspiHandle;
    uint32_t      ui32Index = 0;

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (psMSPISettings == NULL))
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_MX25U12835F_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmMx25u12835f[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_MX25U12835F_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    for ( uint32_t i = 0; i < (sizeof(g_MSPIMx25u12835f_DevConfig) / sizeof(g_MSPIMx25u12835f_DevConfig[0])); i++ )
    {
        if ( (psMSPISettings->eDeviceConfig == g_MSPIMx25u12835f_DevConfig[i].eHalDeviceEnum) )
        {
            psConfig = g_MSPIMx25u12835f_DevConfig[i].psDevConfig;
            psConfig->eClockFreq = psMSPISettings->eClockFreq;
            psConfig->pTCB = psMSPISettings->pNBTxnBuf;
            psConfig->ui32TCBSize = psMSPISettings->ui32NBTxnBufLength;
            psConfig->scramblingStartAddr = psMSPISettings->ui32ScramblingStartAddr;
            psConfig->scramblingEndAddr = psMSPISettings->ui32ScramblingEndAddr;
            break;
        }
    }

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
    switch (psConfig->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
            gAmMx25u12835f[ui32Index].stSetting = MSPI_Mx25u12835f_Serial_CE0_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &MSPI_Mx25u12835f_Serial_CE0_MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            am_bsp_mspi_pins_enable(ui32Module, MSPI_Mx25u12835f_Serial_CE0_MSPIConfig.eDeviceConfig);
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            gAmMx25u12835f[ui32Index].stSetting = MSPI_Mx25u12835f_Serial_CE1_MSPIConfig;
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
            {
                am_util_stdio_printf("Error - Failed to power on MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }

            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, &MSPI_Mx25u12835f_Serial_CE1_MSPIConfig))
            {
                am_util_stdio_printf("Error - Failed to configure MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
            {
                am_util_stdio_printf("Error - Failed to enable MSPI.\n");
                return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            }
            am_bsp_mspi_pins_enable(ui32Module, MSPI_Mx25u12835f_Serial_CE1_MSPIConfig.eDeviceConfig);
            break;
        default:
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
            //break;
    }

    gAmMx25u12835f[ui32Index].pMspiHandle = pMspiHandle;
    gAmMx25u12835f[ui32Index].ui32Module = ui32Module;

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_mx25u12835f_reset((void*)&gAmMx25u12835f[ui32Index]))
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Initialize the MSPI settings for the MSPI_FLASH.
    //
    gAmMx25u12835f[ui32Index].stSetting = *psConfig;

    //
    // Device specific MSPI Mx25u12835f initialization.
    //
    ui32Status = am_device_init_flash((void*)&gAmMx25u12835f[ui32Index]);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    ui32Status = am_hal_mspi_device_configure(pMspiHandle, psConfig);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, psConfig->eDeviceConfig);

    //
    // Enable MSPI interrupts.
    //

    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmMx25u12835f[ui32Index].bOccupied = true;
    *ppMspiHandle = pMspiHandle;
    *ppHandle = (void *)&gAmMx25u12835f[ui32Index];

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief De-Initialization the mspi_mx25u12835f driver.
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
am_devices_mspi_mx25u12835f_deinit(void *pHandle)
{
    uint32_t      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

    //
    // Device specific MSPI Flash de-initialization.
    //
    ui32Status = am_device_deinit_flash(pHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_mx25u12835f_reset(pHandle))
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pFlash->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Disable the MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pFlash->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Deinitialize the MSPI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pFlash->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    // Free this device handle
    pFlash->bOccupied = false;

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
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the current status of the external mx25u12835f
//!
//! @param ui32DeviceNumber - Device number of the external mx25u12835f
//!
//! This function reads the device ID register of the external mx25u12835f, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_reset(void *pHandle)
{
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to reset the device and return status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_RESET_ENABLE, false, 0, ui32PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_RESET_MEMORY, false, 0, ui32PIOBuffer, 0))
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the ID of the external mx25u12835f and returns the value.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external mx25u12835f, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_id(void *pHandle)
{
    uint32_t      ui32Status;
    uint32_t      ui32DeviceID;

    //
    // Send the command sequence to read the Device ID and return status.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_ID, false, 0, &ui32DeviceID, 3);
    am_util_stdio_printf("Flash ID is %8.8X\n", ui32DeviceID);
    if ( ((ui32DeviceID & AM_DEVICES_MSPI_MX25U12835F_ID_MASK) == AM_DEVICES_MSPI_MX25U12835F_ID) &&
       (AM_HAL_STATUS_SUCCESS == ui32Status) )
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
    }
    else
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
}

//*****************************************************************************
//
//! @brief Reads the current status of the external mx25u12835f
//!
//! This function reads the status register of the external mx25u12835f, and returns
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
am_devices_mspi_mx25u12835f_status(void *pHandle, uint32_t *pStatus)
{
    uint32_t      ui32Status;

    //
    // Send the command sequence to read the device status.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_STATUS, false, 0, pStatus, 1);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

uint32_t
am_devices_mspi_mx25u12835f_read_adv(void *pHandle, uint8_t *pui8RxBuffer,
                                     uint32_t ui32ReadAddress,
                                     uint32_t ui32NumBytes,
                                     uint32_t ui32PauseCondition,
                                     uint32_t ui32StatusSetClr,
                                     am_hal_mspi_callback_t pfnCallback,
                                     void *pCallbackCtxt)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

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
    ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                  AM_HAL_MSPI_TRANS_DMA, pfnCallback, pCallbackCtxt);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external mx25u12835f into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the mx25u12835f
//! @param ui32ReadAddress - Address of desired data in external mx25u12835f
//! @param ui32NumBytes - Number of bytes to read from external mx25u12835f
//!
//! This function reads the external mx25u12835f at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_read(void *pHandle, uint8_t *pui8RxBuffer,
                                 uint32_t ui32ReadAddress,
                                 uint32_t ui32NumBytes,
                                 bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

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
        volatile bool bDMAComplete = false;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_MX25U12835F_Callback, (void *)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
        }

        // Check the status.
        if (!bDMAComplete)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external mx25u12835f into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the mx25u12835f
//! @param ui32ReadAddress - Address of desired data in external mx25u12835f
//! @param ui32NumBytes - Number of bytes to read from external mx25u12835f
//!
//! This function reads the external mx25u12835f at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_read_hiprio(void *pHandle, uint8_t *pui8RxBuffer,
                                        uint32_t ui32ReadAddress,
                                        uint32_t ui32NumBytes,
                                        bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    uint32_t                      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

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
        volatile bool bDMAComplete = false;
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_MX25U12835F_Callback, (void*)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
        }

        // Check the status.
        if (!bDMAComplete)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }
    }
    else
    {
        // Check the transaction status.
        ui32Status = am_hal_mspi_highprio_transfer(pFlash->pMspiHandle, &Transaction,
                                                      AM_HAL_MSPI_TRANS_DMA, NULL, NULL);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}



//*****************************************************************************
//
//! @brief Programs the given range of mx25u12835f addresses.
//!
//! @param ui32DeviceNumber - Device number of the external mx25u12835f
//! @param pui8TxBuffer - Buffer to write the external mx25u12835f data from
//! @param ui32WriteAddress - Address to write to in the external mx25u12835f
//! @param ui32NumBytes - Number of bytes to write to the external mx25u12835f
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external mx25u12835f at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target mx25u12835f
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_write(void *pHandle, uint8_t *pui8TxBuffer,
                                  uint32_t ui32WriteAddress,
                                  uint32_t ui32NumBytes,
                                  bool bWaitForCompletion)
{
    am_hal_mspi_dma_transfer_t    Transaction;
    bool                          bWriteComplete = false;
    uint32_t                      ui32BytesLeft = ui32NumBytes;
    uint32_t                      ui32PageAddress = ui32WriteAddress;
    uint32_t                      ui32BufferAddress = (uint32_t)pui8TxBuffer;
    uint32_t                      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    while (ui32BytesLeft > 0)
    {
        //
        // Send the command sequence to enable writing.
        //
        ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }

        // Set the DMA priority
        Transaction.ui8Priority = 1;

        // Set the transfer direction to TX (Write)
        Transaction.eDirection = AM_HAL_MSPI_TX;

        if (ui32BytesLeft > AM_DEVICES_MSPI_MX25U12835F_PAGE_SIZE)
        {
            // Set the transfer count in bytes.
            Transaction.ui32TransferCount = AM_DEVICES_MSPI_MX25U12835F_PAGE_SIZE;
            ui32BytesLeft -= AM_DEVICES_MSPI_MX25U12835F_PAGE_SIZE;
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
        ui32PageAddress += AM_DEVICES_MSPI_MX25U12835F_PAGE_SIZE;

        // Set the source SRAM buffer address.
        Transaction.ui32SRAMAddress = ui32BufferAddress;
        ui32BufferAddress += AM_DEVICES_MSPI_MX25U12835F_PAGE_SIZE;

        // Clear the CQ stimulus.
        Transaction.ui32PauseCondition = 0;
        // Clear the post-processing
        Transaction.ui32StatusSetClr = 0;

        // Start the transaction.
        volatile bool bDMAComplete = false;
        ui32Status = am_hal_mspi_nonblocking_transfer(pFlash->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA, pfnMSPI_MX25U12835F_Callback, (void*)&bDMAComplete);

        // Check the transaction status.
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }

        // Wait for DMA Complete or Timeout
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_TIMEOUT; i++)
        {
            if (bDMAComplete)
            {
                break;
            }
            //
            // Call the BOOTROM cycle function to delay for about 1 microsecond.
            //
            am_hal_flash_delay( FLASH_CYCLES_US(1) );
        }

        // Check the status.
        if (!bDMAComplete)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }

        //
        // Wait for the Write In Progress to indicate the erase is complete.
        //
        for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_TIMEOUT; i++)
        {
          ui32Status = am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_STATUS, false, 0, ui32PIOBuffer, 1);
          if ((AM_HAL_MSPI_FLASH_QUADPAIRED == pFlash->stSetting.eDeviceConfig) ||
              (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == pFlash->stSetting.eDeviceConfig))
          {
            bWriteComplete = ((0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP)) &&
                              (0 == ((ui32PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_MX25U12835F_WIP)));
          }
          else
          {
            bWriteComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP));
          }
          am_util_delay_us(100);
          if (bWriteComplete)
          {
            break;
          }
        }

        //
        // Send the command sequence to disable writing.
        //
        ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
        }
    }

  //
  // Return the status.
  //
  return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Erases the entire contents of the external mx25u12835f
//!
//! @param ui32DeviceNumber - Device number of the external mx25u12835f
//!
//! This function uses the "Bulk Erase" instruction to erase the entire
//! contents of the external mx25u12835f.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_mass_erase(void *pHandle)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the mass erase.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_BULK_ERASE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_ERASE_TIMEOUT; i++)
    {
        ui32PIOBuffer[0] = 0;
        am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_STATUS, false, 0, ui32PIOBuffer, 1);
        if ((AM_HAL_MSPI_FLASH_QUADPAIRED == pFlash->stSetting.eDeviceConfig) ||
            (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == pFlash->stSetting.eDeviceConfig))
        {
            bEraseComplete = ((0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP)) &&
                              (0 == ((ui32PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_MX25U12835F_WIP)));
        }
        else
        {
            bEraseComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP));
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
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Erases the contents of a single sector of mx25u12835f
//!
//! @param ui32DeviceNumber - Device number of the external mx25u12835f
//! @param ui32SectorAddress - Address to erase in the external mx25u12835f
//!
//! This function erases a single sector of the external mx25u12835f as specified by
//! ui32EraseAddress. The entire sector where ui32EraseAddress will
//! be erased.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_sector_erase(void *pHandle, uint32_t ui32SectorAddress)
{
    bool          bEraseComplete = false;
    uint32_t      ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command sequence to enable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Send the command sequence to do the sector erase.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_SECTOR_ERASE, true, ui32SectorAddress, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Wait for the Write In Progress to indicate the erase is complete.
    //
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_MX25U12835F_ERASE_TIMEOUT; i++)
    {
        ui32PIOBuffer[0] = 0;
        am_devices_mspi_mx25u12835f_command_read(pHandle, AM_DEVICES_MSPI_MX25U12835F_READ_STATUS, false, 0, ui32PIOBuffer, 1);
        if ((AM_HAL_MSPI_FLASH_QUADPAIRED == pFlash->stSetting.eDeviceConfig) ||
            (AM_HAL_MSPI_FLASH_QUADPAIRED_SERIAL == pFlash->stSetting.eDeviceConfig))
        {
            bEraseComplete = ((0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP)) &&
                              (0 == ((ui32PIOBuffer[0] >> 8) & AM_DEVICES_MSPI_MX25U12835F_WIP)));
        }
        else
        {
            bEraseComplete = (0 == (ui32PIOBuffer[0] & AM_DEVICES_MSPI_MX25U12835F_WIP));
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
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Send the command sequence to disable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_DISABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Sets up the MSPI and external MX25U12835F into XIP mode.
//!
//! This function sets the external psram device and the MSPI into XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_enable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

    //
    // Enable XIP on the MSPI.
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

#if !MSPI_USE_CQ
    // Disable the DMA interrupts.
    ui32Status = am_hal_mspi_interrupt_disable(pFlash->pMspiHandle,
                                               AM_HAL_MSPI_INT_DMAERR |
                                               AM_HAL_MSPI_INT_DMACMP );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
#endif

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Removes the MSPI and external MX25U12835F from XIP mode.
//!
//! This function removes the external device and the MSPI from XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_disable_xip(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command to enable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Disable XIP on the MSPI.
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Sets up the MSPI and external MX25U12835F into scrambling mode.
//!
//! This function sets the external psram device and the MSPI into scrambling mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_enable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;

    //
    // Enable scrambling on the MSPI.
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }
    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Removes the MSPI and external MX25U12835F from scrambling mode.
//!
//! This function removes the external device and the MSPI from scrambling mode.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_mx25u12835f_disable_scrambling(void *pHandle)
{
    uint32_t ui32Status;
    am_devices_mspi_mx25u12835f_t *pFlash = (am_devices_mspi_mx25u12835f_t *)pHandle;
    uint32_t      ui32PIOBuffer[32] = {0};

    //
    // Send the command to enable writing.
    //
    ui32Status = am_devices_mspi_mx25u12835f_command_write(pHandle, AM_DEVICES_MSPI_MX25U12835F_WRITE_ENABLE, false, 0, ui32PIOBuffer, 0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    //
    // Disable Scrambling on the MSPI.
    //
    ui32Status = am_hal_mspi_control(pFlash->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_MX25U12835F_STATUS_ERROR;
    }

    return AM_DEVICES_MSPI_MX25U12835F_STATUS_SUCCESS;
}

