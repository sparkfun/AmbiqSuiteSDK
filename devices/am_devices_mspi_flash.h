//*****************************************************************************
//
//! @file am_devices_mspi_flash.h
//!
//! @brief Micron Serial NOR SPI Flash driver.
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

#ifndef AM_DEVICES_MSPI_FLASH_H
#define AM_DEVICES_MSPI_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions for flash commands
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_WRITE_STATUS      0x01
#define AM_DEVICES_MSPI_FLASH_PAGE_PROGRAM      0x02
#define AM_DEVICES_MSPI_FLASH_READ              0x03
#define AM_DEVICES_MSPI_FLASH_WRITE_DISABLE     0x04
#define AM_DEVICES_MSPI_FLASH_READ_STATUS       0x05
#define AM_DEVICES_MSPI_FLASH_WRITE_ENABLE      0x06
#define AM_DEVICES_MSPI_FLASH_FAST_READ         0x0B
#define AM_DEVICES_MSPI_FLASH_READ_4B           0x13
#define AM_DEVICES_MSPI_FLASH_SUBSECTOR_ERASE   0x20
#define AM_DEVICES_MSPI_FLASH_DUAL_READ         0x3B
#define AM_DEVICES_MSPI_FLASH_DUAL_IO_READ      0xBB
#define AM_DEVICES_MSPI_FLASH_WRITE_ENHVOL_CFG  0x61
#define AM_DEVICES_MSPI_FLASH_RESET_ENABLE      0x66
#define AM_DEVICES_MSPI_FLASH_QUAD_READ         0x6B
#define AM_DEVICES_MSPI_FLASH_WRITE_VOL_CFG     0x81
#define AM_DEVICES_MSPI_FLASH_RESET_MEMORY      0x99
#define AM_DEVICES_MSPI_FLASH_READ_ID           0x9F
#define AM_DEVICES_MSPI_FLASH_ENTER_4B          0xB7
#define AM_DEVICES_MSPI_FLASH_BULK_ERASE        0xC7
#define AM_DEVICES_MSPI_FLASH_SECTOR_ERASE      0xD8
#define AM_DEVICES_MSPI_FLASH_EXIT_4B           0xE9
#define AM_DEVICES_MSPI_FLASH_QUAD_IO_READ      0xEB
#define AM_DEVICES_MSPI_FLASH_READ_QUAD_4B      0xEC

//*****************************************************************************
//
// Global definitions for the flash status register
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_WEL         0x00000002        // Write enable latch
#define AM_DEVICES_MSPI_FLASH_WIP         0x00000001        // Write in progress

//
// The following definitions are typically specific to a multibit spi flash device.
// They should be tailored by the example or testcase (i.e., defined in the project).
//
//#define MICRON_N25Q256A
//#define CYPRESS_S25FS064S
//#define MACRONIX_MX25U12835F
//#define ADESTO_ATXP032

#if defined (MICRON_N25Q256A)
//*****************************************************************************
//
// Device specific identification.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_ID            0x0019BA20
#define AM_DEVICES_MSPI_FLASH_ID_MASK       0x00FFFFFF

//*****************************************************************************
//
// Device specific definitions for flash commands
//
//*****************************************************************************
// None.
//*****************************************************************************
//
// Device specific definitions for the Configuration register(s)
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_DUMMY_CLOCKS(n)     (((uint8_t)(n) << 4) & 0xF0)
#define AM_DEVICES_MSPI_FLASH_XIP(n)              (((uint8_t)(n) << 3) & 0x08)
#define AM_DEVICES_MSPI_FLASH_WRAP(n)             (((uint8_t)(n)) & 0x03)
#define AM_DEVICES_MSPI_FLASH_QUAD_MODE           (0x0A)
#define AM_DEVICES_MSPI_FLASH_DUAL_MODE           (0x8A)
#define AM_DEVICES_MSPI_FLASH_SERIAL_MODE         (0xCA)

//*****************************************************************************
//
// Device specific definitions for the flash size information
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_PAGE_SIZE       0x100    //256 bytes, minimum program unit
#define AM_DEVICES_MSPI_FLASH_SUBSECTOR_SIZE  0x1000   //4K bytes
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE     0x10000  //64K bytes
#define AM_DEVICES_MSPI_FLASH_MAX_SECTORS     256      // Sectors within 3-byte address range.
#endif

#if defined (CYPRESS_S25FS064S)
//*****************************************************************************
//
// Device specific identification.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_ID        0x00170201
#define AM_DEVICES_MSPI_FLASH_ID_MASK   0x00FFFFFF

//*****************************************************************************
//
// Device specific definitions for flash commands
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_READ_STATUS2              0x07
#define AM_DEVICES_MSPI_FLASH_WRITE_ANY_REG             0x71
#define AM_DEVICES_MSPI_FLASH_EVAL_ERASE_STATUS         0xD0

//*****************************************************************************
//
// Device specific definitions for the Configuration register(s)
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_QUAD_MODE           (0x02)
#define AM_DEVICES_MSPI_FLASH_DUAL_MODE           (0x00)
#define AM_DEVICES_MSPI_FLASH_SERIAL_MODE         (0x00)
#define AM_DEVICES_MSPI_FLASH_CR1V                (0x00020080)
#define AM_DEVICES_MSPI_FLASH_CR1V_VALUE(n)       (((uint32_t)(n) << 24) & 0xFF000000)
#define AM_DEVICES_MSPI_FLASH_CR2V                (0x00030080)
#define AM_DEVICES_MSPI_FLASH_CR2V_VALUE(n)       (((uint32_t)(n) << 24) & 0xFF000000)
#define AM_DEVICES_MSPI_FLASH_ADDR_QUAD_EXT_DELAY (2)
#define AM_DEVICES_MSPI_FLASH_ADDR_DUAL_EXT_DELAY (4)
#define AM_DEVICES_MSPI_FLASH_CR3V                (0x00040080)
#define AM_DEVICES_MSPI_FLASH_CR4V                (0x00050080)
#define AM_DEVICES_MSPI_FLASH_CR3V_VALUE          (((uint32_t)(0x08) << 24) & 0xFF000000)
#define AM_DEVICES_MSPI_FLASH_CR4V_VALUE          (((uint32_t)(0x10) << 24) & 0xFF000000)
#define AM_DEVICES_MSPI_FLASH_ERASE_SUCCESS       (0x04)

//*****************************************************************************
//
// Device specific definitions for the flash size information
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_PAGE_SIZE       0x100    //256 bytes, minimum program unit
#define AM_DEVICES_MSPI_FLASH_SUBSECTOR_SIZE  0x1000   //4K bytes
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE     0x10000  //64K bytes.
#define AM_DEVICES_MSPI_FLASH_MAX_SECTORS     128      // Sectors within 3-byte address range.
#endif

#if defined (MACRONIX_MX25U12835F)
//*****************************************************************************
//
// Device specific identification.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_ID        0x003825C2
#define AM_DEVICES_MSPI_FLASH_ID_MASK   0x00FFFFFF

//*****************************************************************************
//
// Device specific definitions for flash commands
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_READ_CONFIG               0x15
#define AM_DEVICES_MSPI_FLASH_ENABLE_QPI_MODE           0x35
#define AM_DEVICES_MSPI_FLASH_DISABLE_QPI_MODE          0xF5

//*****************************************************************************
//
// Device specific definitions for the Configuration register(s)
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_STATUS              (0x00)
#define AM_DEVICES_MSPI_FLASH_CONFIG              (0x07)

//*****************************************************************************
//
// Device specific definitions for the flash size information
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_PAGE_SIZE       0x100    //256 bytes, minimum program unit
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE     0x1000   //4K bytes
#define AM_DEVICES_MSPI_FLASH_BLOCK_SIZE      0x10000  //64K bytes.
#define AM_DEVICES_MSPI_FLASH_MAX_SECTORS     4096     // Sectors within 3-byte address range.
#define AM_DEVICES_MSPI_FLASH_MAX_BLOCKS      256
#endif

#if defined (ADESTO_ATXP032)
//*****************************************************************************
//
// Device specific identification.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_ID        0x0043A700
#define AM_DEVICES_MSPI_FLASH_ID_MASK   0x00FFFFFF
//*****************************************************************************
//
// Device specific definitions for flash commands
//
//*****************************************************************************
#define AM_DEVICES_ATXP032_ENTER_QUAD_MODE      0x38
#define AM_DEVICES_ATXP032_UNPROTECT_SECTOR     0x39
#define AM_DEVICES_ATXP032_WRITE_STATUS_CTRL    0x71
#define AM_DEVICES_ATXP032_ENTER_OCTAL_MODE     0xE8
#define AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE   0xFF
//*****************************************************************************
//
// Device specific definitions for the flash size information
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_PAGE_SIZE       0x100    //256 bytes, minimum program unit
//#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE     0x10000   //64K bytes
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE     0x1000   //4K bytes
#define AM_DEVICES_MSPI_FLASH_MAX_BLOCKS      256
#define AM_DEVICES_MSPI_FLASH_MAX_SECTORS     256      // Sectors within 4-byte address range.

//*****************************************************************************
//
// Global definitions for the flash status register
//
//*****************************************************************************
#define AM_DEVICES_ATXP032_RSTE        0x00000010        // Reset enable
#define AM_DEVICES_ATXP032_WEL         0x00000002        // Write enable latch
#define AM_DEVICES_ATXP032_WIP         0x00000001        // Operation in progress
  
#endif

//*****************************************************************************
//
// Global definitions for the MSPI instance to use.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_FLASH_MSPI_INSTANCE     0

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS,
    AM_DEVICES_MSPI_FLASH_STATUS_ERROR
} am_devices_mspi_flash_status_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern volatile uint32_t g_MSPIInterruptStatus;

extern uint32_t am_devices_mspi_flash_init(uint32_t ui32Module,
                                           am_hal_mspi_dev_config_t *psMSPISettings,
                                           void **pHandle);

extern uint32_t am_devices_mspi_flash_deinit(uint32_t ui32Module, am_hal_mspi_dev_config_t *psMSPISettings);

extern uint32_t am_devices_mspi_flash_id(uint32_t ui32Module);

extern uint32_t am_devices_mspi_flash_reset(uint32_t ui32Module);

extern uint32_t am_devices_mspi_flash_status(uint32_t ui32Module, uint32_t *pStatus);

extern uint32_t am_devices_mspi_flash_read(uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           bool bWaitForCompletion);

extern uint32_t am_devices_mspi_flash_write(uint32_t ui32Module, uint8_t *ui8TxBuffer,
                                            uint32_t ui32WriteAddress,
                                            uint32_t ui32NumBytes);

extern uint32_t am_devices_mspi_flash_mass_erase(uint32_t ui32Module);

extern uint32_t am_devices_mspi_flash_sector_erase(uint32_t ui32Module, uint32_t ui32SectorAddress);

extern uint32_t am_devices_mspi_flash_enable(bool bEnableXIP, bool bEnableScrambling);

extern uint32_t am_devices_mspi_flash_disable_xip(uint32_t ui32Module);

extern uint32_t
am_devices_mspi_flash_read_adv(uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt);

extern am_hal_mspi_dev_config_t 
am_devices_mspi_flash_mode_switch(uint32_t ui32Module,
                                  am_hal_mspi_dev_config_t *pMSPISettings);

extern void am_devices_mspi_mixmode_switch(uint32_t ui32Mode);
extern uint32_t
am_devices_mspi_flash_read_hiprio(uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);



#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_FLASH_H

