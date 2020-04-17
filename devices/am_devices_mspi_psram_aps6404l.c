//*****************************************************************************
//
//! @file am_devices_mspi_psram.c
//!
//! @brief General Multibit SPI PSRAM driver.
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

#include "am_mcu_apollo.h"
#include "am_devices_mspi_psram_aps6404l.h"
#include "am_util_stdio.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_TIMEOUT             1000000

am_hal_mspi_dev_config_t  SerialCE0MSPIConfig =
{
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .eXipMixedMode        = AM_HAL_MSPI_XIPMIXED_NORMAL,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bSeparateIO          = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#endif
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  SerialCE1MSPIConfig =
{
  .ui8TurnAround        = 8,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_FAST_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_SERIAL_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .eXipMixedMode        = AM_HAL_MSPI_XIPMIXED_NORMAL,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bSeparateIO          = true,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 80,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#endif
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  QuadCE0MSPIConfig =
{
  .ui8TurnAround        = 6,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE0,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .eXipMixedMode        = AM_HAL_MSPI_XIPMIXED_NORMAL,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bSeparateIO          = false,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 30,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#endif
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

am_hal_mspi_dev_config_t  QuadCE1MSPIConfig =
{
  .ui8TurnAround        = 6,
  .eAddrCfg             = AM_HAL_MSPI_ADDR_3_BYTE,
  .eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE,
  .ui8ReadInstr         = AM_DEVICES_MSPI_PSRAM_QUAD_READ,
  .ui8WriteInstr        = AM_DEVICES_MSPI_PSRAM_QUAD_WRITE,
  .eDeviceConfig        = AM_HAL_MSPI_FLASH_QUAD_CE1,
  .eSpiMode             = AM_HAL_MSPI_SPI_MODE_0,
  .eClockFreq           = AM_HAL_MSPI_CLK_24MHZ,
  .eXipMixedMode        = AM_HAL_MSPI_XIPMIXED_NORMAL,
  .bSendAddr            = true,
  .bSendInstr           = true,
  .bSeparateIO          = false,
  .bTurnaround          = true,
#if defined(AM_PART_APOLLO3P)
  .ui8WriteLatency      = 0,
  .bEnWriteLatency      = false,
  .bEmulateDDR          = false,
  .ui16DMATimeLimit     = 30,
  .eDMABoundary         = AM_HAL_MSPI_BOUNDARY_BREAK1K,
#endif
  .ui32TCBSize          = 0,
  .pTCB                 = NULL,
  .scramblingStartAddr  = 0,
  .scramblingEndAddr    = 0,
};

typedef struct
{
  uint32_t                    ui32Module;
  void                        *pMspiHandle;
  am_hal_mspi_device_e        eDeviceConfig;
#if defined(AM_PART_APOLLO3)
  uint32_t                    maxTransSize;
#endif
  bool                        bOccupied;
} am_devices_mspi_psram_t;

am_devices_mspi_psram_t gAmPsram[AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM];

#if defined(AM_PART_APOLLO3)
// Hardware does not support transaction splitting - so need to take care in driver
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
#endif

void pfnMSPI_PSRAM_Callback(void *pCallbackCtxt, uint32_t status)
{
  // Set the DMA complete flag.
  *(volatile bool *)pCallbackCtxt = true;
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_device_command_write(void *pMspiHandle,
                        uint8_t ui8Instr,
                        bool bSendAddr,
                        uint32_t ui32Addr,
                        uint32_t *pData,
                        uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_TX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;
#endif
  Transaction.bQuadCmd                = false;
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_device_command_read(void *pMspiHandle,
                       uint8_t ui8Instr,
                       bool bSendAddr,
                       uint32_t ui32Addr,
                       uint32_t *pData,
                       uint32_t ui32NumBytes)
{
  am_hal_mspi_pio_transfer_t  Transaction;

  // Create the individual write transaction.
  Transaction.ui32NumBytes            = ui32NumBytes;
  Transaction.bScrambling             = false;
  Transaction.eDirection              = AM_HAL_MSPI_RX;
  Transaction.bSendAddr               = bSendAddr;
  Transaction.ui32DeviceAddr          = ui32Addr;
  Transaction.bSendInstr              = true;
  Transaction.ui16DeviceInstr         = ui8Instr;
  Transaction.bTurnaround             = false;
#if defined(AM_PART_APOLLO3P)
  Transaction.bDCX                    = false;
  Transaction.bEnWRLatency            = false;
  Transaction.bContinue               = false;
#endif
  Transaction.bQuadCmd                = false;
  Transaction.pui32Buffer             = pData;

  // Execute the transction over MSPI.
  return am_hal_mspi_blocking_transfer(pMspiHandle,
                                       &Transaction,
                                       AM_DEVICES_MSPI_PSRAM_TIMEOUT);
}

//*****************************************************************************
//
// Reset the external psram
//
//*****************************************************************************
static uint32_t
am_devices_mspi_psram_aps6404l_reset(void *pMspiHandle)
{
  uint32_t      ui32PIOBuffer;
  //
  // Send the command sequence to reset the device and return status.
  //
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_RESET_ENABLE, false, 0, &ui32PIOBuffer, 0))
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }
  if (AM_HAL_STATUS_SUCCESS != am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_RESET_MEMORY, false, 0, &ui32PIOBuffer, 0))
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
static uint32_t
am_devices_mspi_psram_aps6404l_id(void *pMspiHandle)
{
  uint32_t      ui32Status;
  uint32_t aui32Rawdata[2] = {0};
  uint32_t      ui32DeviceID = 0;

  //
  // Send the command sequence to read the Device ID and return status.
  //
  ui32Status = am_device_command_read(pMspiHandle, AM_DEVICES_MSPI_PSRAM_READ_ID, false, 0, aui32Rawdata, 5);
  ui32DeviceID = ((aui32Rawdata[0] & 0xFF000000) >> 24) | ((aui32Rawdata[1] & 0xFF) << 8);
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

// This function takes care of splitting the transaction as needed, if the transaction crosses
// PSRAM page boundary or because of tCEM restrictions, if hardware does not support it
static uint32_t
psram_nonblocking_transfer(am_devices_mspi_psram_t *pPsram,
                           bool bHiPrio,
                           bool bWrite,
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


  // Set the transfer direction to RX (Read)
  Transaction.eDirection = bWrite ? AM_HAL_MSPI_TX: AM_HAL_MSPI_RX;


  // Initialize the CQ stimulus.
  Transaction.ui32PauseCondition = ui32PauseCondition;
  // Initialize the post-processing
  Transaction.ui32StatusSetClr = 0;

  // Need to be aware of page size
  while (ui32NumBytes)
  {
#if defined(AM_PART_APOLLO3)
    uint32_t maxSize = AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - 1));
    uint32_t limit = (maxSize > pPsram->maxTransSize) ? pPsram->maxTransSize : maxSize;
    uint32_t size = (ui32NumBytes > limit) ? limit : ui32NumBytes;
#else
    uint32_t size;
    if ((ui32Address & 0x3) &&
        ((AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - (ui32Address & (AM_DEVICES_MSPI_PSRAM_PAGE_SIZE - 1))) < ui32NumBytes))
    {
      // Hardware does not support Page splitting if address is not word aligned
      // Need to split the transaction
      size = 4 - (ui32Address & 0x3);
    }
    else
    {
      size = ui32NumBytes;
    }
#endif
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
      ui32Status = am_hal_mspi_highprio_transfer(pPsram->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
                                                 bLast ? pfnCallback : NULL,
                                                 bLast ? pCallbackCtxt : NULL);
    }
    else
    {
      ui32Status = am_hal_mspi_nonblocking_transfer(pPsram->pMspiHandle, &Transaction, AM_HAL_MSPI_TRANS_DMA,
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
am_devices_mspi_psram_init(uint32_t ui32Module, am_devices_mspi_psram_config_t *pDevCfg, void **ppHandle, void **ppMspiHandle)
{
    uint32_t                    ui32Status;
    uint32_t                    ui32PIOBuffer;
    am_hal_mspi_dev_config_t    *psMSPISettings1;
    am_hal_mspi_dev_config_t    *psMSPISettings0;
    am_hal_mspi_dev_config_t    mspiDevCfg;
    void                        *pMspiHandle;
    uint32_t                    ui32Index = 0;

    if ((ui32Module > AM_REG_MSPI_NUM_MODULES) || (pDevCfg == NULL))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Enable fault detection.
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmPsram[ui32Index].bOccupied == false )
        {
            break;
        }
    }
    if ( ui32Index == AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Re-Configure the MSPI for the requested operation mode.
    //
    switch (pDevCfg->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
            mspiDevCfg = SerialCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            mspiDevCfg = SerialCE1MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
            mspiDevCfg = QuadCE0MSPIConfig;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            mspiDevCfg = QuadCE1MSPIConfig;
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    mspiDevCfg.eClockFreq = pDevCfg->eClockFreq;
    mspiDevCfg.ui32TCBSize = pDevCfg->ui32NBTxnBufLength;
    mspiDevCfg.pTCB = pDevCfg->pNBTxnBuf;
    mspiDevCfg.scramblingStartAddr = pDevCfg->ui32ScramblingStartAddr;
    mspiDevCfg.scramblingEndAddr = pDevCfg->ui32ScramblingEndAddr;
    mspiDevCfg.eXipMixedMode = pDevCfg->eMixedMode;

#if defined(AM_PART_APOLLO3)
    //
    // Look up the Max Transaction size to fit into 8usec for CE asserted
    //
    gAmPsram[ui32Index].maxTransSize = 0;
    for (uint32_t i = 0; i < (sizeof(g_Mspi_SpeedMax) / sizeof(g_Mspi_SpeedMax[0])); i++)
    {
        if (g_Mspi_SpeedMax[i].MHz == mspiDevCfg.eClockFreq)
        {
            gAmPsram[ui32Index].maxTransSize = g_Mspi_SpeedMax[i].MaxSize;
            break;
        }
    }
    if ( 0 == gAmPsram[ui32Index].maxTransSize ) // Return an error if Max Transaction size not found.
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
#endif

    //
    // Configure the MSPI for Serial or Quad operation during initialization.
    //
    switch (mspiDevCfg.eDeviceConfig)
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
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }
    psMSPISettings0->eXipMixedMode = pDevCfg->eMixedMode;

    // First configure in Quad mode and reset
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_initialize(ui32Module, &pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to initialize MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pMspiHandle, AM_HAL_SYSCTRL_WAKE, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, psMSPISettings0))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI device.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings0->eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps6404l_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Revert to Serial mode
    switch (mspiDevCfg.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            //break;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_device_configure(pMspiHandle, psMSPISettings1))
    {
        am_util_stdio_printf("Error - Failed to configure MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_enable(pMspiHandle))
    {
        am_util_stdio_printf("Error - Failed to enable MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    am_bsp_mspi_pins_enable(ui32Module, psMSPISettings1->eDeviceConfig);

    am_util_delay_us(150);

    if (AM_HAL_STATUS_SUCCESS != am_devices_mspi_psram_aps6404l_reset(pMspiHandle))
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Device specific MSPI psram initialization.
    //
    ui32Status = am_devices_mspi_psram_aps6404l_id(pMspiHandle);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the APS6404L Device mode.
    //
    switch (mspiDevCfg.eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_device_command_write(pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_ENTER, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Disable MSPI defore re-configuring it
    ui32Status = am_hal_mspi_disable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_device_configure(pMspiHandle, &mspiDevCfg);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Re-Enable MSPI
    ui32Status = am_hal_mspi_enable(pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Configure the MSPI pins.
    //
    am_bsp_mspi_pins_enable(ui32Module, mspiDevCfg.eDeviceConfig);

    //
    // Enable MSPI interrupts.
    //

    ui32Status = am_hal_mspi_interrupt_clear(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    ui32Status = am_hal_mspi_interrupt_enable(pMspiHandle, AM_HAL_MSPI_INT_CQUPD | AM_HAL_MSPI_INT_ERR );
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Return the handle.
    //
    gAmPsram[ui32Index].bOccupied = true;
    *ppHandle = (void *)&gAmPsram[ui32Index];
    *ppMspiHandle = gAmPsram[ui32Index].pMspiHandle = pMspiHandle;
    gAmPsram[ui32Index].ui32Module = ui32Module;
    gAmPsram[ui32Index].eDeviceConfig = mspiDevCfg.eDeviceConfig;

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
am_devices_mspi_psram_deinit(void *pHandle)
{
    uint32_t    ui32Status;
    uint32_t    ui32PIOBuffer;
    am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

    //
    // Device specific MSPI psram initialization.
    //
    // Revert to Serial mode
    switch (pPsram->eDeviceConfig)
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
            // Nothing to do.  Device defaults to SPI mode.
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            ui32Status = am_device_command_write(pPsram->pMspiHandle, AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT, false, 0, &ui32PIOBuffer, 0);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
            }
            break;
        default:
            return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
        //break;
    }

    //
    // Disable and clear the interrupts to start with.
    //
    ui32Status = am_hal_mspi_interrupt_disable(pPsram->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    ui32Status = am_hal_mspi_interrupt_clear(pPsram->pMspiHandle, 0xFFFFFFFF);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    //
    // Disable MSPI instance.
    //
    ui32Status = am_hal_mspi_disable(pPsram->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Disable power to the MSPI instance.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_mspi_power_control(pPsram->pMspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false))
    {
        am_util_stdio_printf("Error - Failed to power on MSPI.\n");
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
    //
    // Deinitialize the MPSI instance.
    //
    ui32Status = am_hal_mspi_deinitialize(pPsram->pMspiHandle);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Free this device handle
    pPsram->bOccupied = false;

    //
    // Return the status.
    //
    return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param ui32Module - MSPI instance
//! @param pui8RxBuffer - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress - Address of desired data in external PSRAM
//! @param ui32NumBytes - Number of bytes to read from external PSRAM
//! @param bWaitForCompletion - Wait for transaction completion before exiting
//!
//! This function reads the external PSRAM at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.  If the bWaitForCompletion is true,
//! then the function will poll for DMA completion indication flag before
//! returning.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read(void *pHandle,
                           uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  if (bWaitForCompletion)
  {
    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                            pui8RxBuffer,
                                            ui32ReadAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            pfnMSPI_PSRAM_Callback,
                                            (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
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
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }
  else
  {
    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(pPsram, false, false,
                                            pui8RxBuffer,
                                            ui32ReadAddress,
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

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param ui32Module - MSPI instance
//! @param pui8RxBuffer - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress - Address of desired data in external PSRAM
//! @param ui32NumBytes - Number of bytes to read from external PSRAM
//! @param ui32PauseCondition - Pause condition before transaction is executed
//! @param ui32StatusSetClr - Post-transaction CQ condition
//! @param pfnCallback - Post-transaction callback function
//! @param pCallbackCtxt - Post-transaction callback context
//!
//! This function reads the external PSRAM at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.  The Command Queue pre and post
//! transaction conditions and a callback function and context are also
//! provided.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_read_adv(void *pHandle,
                               uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, false, false,
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
am_devices_mspi_psram_read_hiprio(void *pHandle,
                                  uint8_t *pui8RxBuffer,
                                  uint32_t ui32ReadAddress,
                                  uint32_t ui32NumBytes,
                                  am_hal_mspi_callback_t pfnCallback,
                                  void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, true, false,
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

uint32_t
am_devices_mspi_psram_nonblocking_read(void *pHandle,
                                       uint8_t *pui8RxBuffer,
                                       uint32_t ui32ReadAddress,
                                       uint32_t ui32NumBytes,
                                       am_hal_mspi_callback_t pfnCallback,
                                       void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, false, false,
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
am_devices_mspi_psram_write(void *pHandle,
                            uint8_t *pui8TxBuffer,
                            uint32_t ui32WriteAddress,
                            uint32_t ui32NumBytes,
                            bool bWaitForCompletion)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  if (bWaitForCompletion)
  {
    // Start the transaction.
    volatile bool bDMAComplete = false;
    ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                            pui8TxBuffer,
                                            ui32WriteAddress,
                                            ui32NumBytes,
                                            0,
                                            0,
                                            pfnMSPI_PSRAM_Callback,
                                            (void *)&bDMAComplete);

    // Check the transaction status.
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }

    // Wait for DMA Complete or Timeout
    for (uint32_t i = 0; i < AM_DEVICES_MSPI_PSRAM_TIMEOUT; i++)
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
      return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
    }
  }
  else
  {
    // Check the transaction status.
    ui32Status = psram_nonblocking_transfer(pPsram, false, true,
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
am_devices_mspi_psram_write_adv(void *pHandle,
                                uint8_t *puiTxBuffer,
                                uint32_t ui32WriteAddress,
                                uint32_t ui32NumBytes,
                                uint32_t ui32PauseCondition,
                                uint32_t ui32StatusSetClr,
                                am_hal_mspi_callback_t pfnCallback,
                                void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  ui32Status = psram_nonblocking_transfer(pPsram, false, true,
                                          puiTxBuffer,
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
am_devices_mspi_psram_write_hiprio(void *pHandle,
                                   uint8_t *pui8TxBuffer,
                                   uint32_t ui32WriteAddress,
                                   uint32_t ui32NumBytes,
                                   am_hal_mspi_callback_t pfnCallback,
                                   void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, true, true,
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
uint32_t
am_devices_mspi_psram_nonblocking_write(void *pHandle,
                                        uint8_t *pui8TxBuffer,
                                        uint32_t ui32WriteAddress,
                                        uint32_t ui32NumBytes,
                                        am_hal_mspi_callback_t pfnCallback,
                                        void *pCallbackCtxt)
{
  uint32_t                      ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  // Check the transaction status.
  ui32Status = psram_nonblocking_transfer(pPsram, false, true,
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
am_devices_mspi_psram_enable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Enable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_EN, NULL);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

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
am_devices_mspi_psram_disable_xip(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable XIP on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, NULL);
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
am_devices_mspi_psram_enable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Enable scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_EN, NULL);
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
am_devices_mspi_psram_disable_scrambling(void *pHandle)
{
  uint32_t ui32Status;
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  //
  // Disable Scrambling on the MSPI.
  //
  ui32Status = am_hal_mspi_control(pPsram->pMspiHandle, AM_HAL_MSPI_REQ_SCRAMB_DIS, NULL);
  if (AM_HAL_STATUS_SUCCESS != ui32Status)
  {
    return AM_DEVICES_MSPI_PSRAM_STATUS_ERROR;
  }

  return AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the external psram
//
//*****************************************************************************
uint32_t
am_devices_mspi_psram_reset(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;
  return am_devices_mspi_psram_aps6404l_reset(pPsram);
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
am_devices_mspi_psram_id(void *pHandle)
{
  am_devices_mspi_psram_t *pPsram = (am_devices_mspi_psram_t *)pHandle;

  return am_devices_mspi_psram_aps6404l_id(pPsram);
}

