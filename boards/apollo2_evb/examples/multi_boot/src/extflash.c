//*****************************************************************************
//
//! @file extflash.c
//!
//! @brief This file provides the external flash handling interfaces
//!
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
#include "am_devices.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_multi_boot.h"
#include "extflash.h"

//*****************************************************************************
//
// External Flash handling
//
//*****************************************************************************
#ifndef AM_BSP_FLASH_IOM
#define AM_BSP_FLASH_IOM    0
#endif

#ifndef AM_BSP_FLASH_CS
#define AM_BSP_FLASH_CS     4
#endif

#ifndef AM_BSP_GPIO_FLASH_CS
#define AM_BSP_GPIO_FLASH_CS     31
#endif

#ifndef AM_BSP_GPIO_CFG_FLASH_CS
#define AM_BSP_GPIO_CFG_FLASH_CS     AM_HAL_PIN_31_M0nCE4
#endif
//*****************************************************************************
//
// Device structure for the SPI flash.
//
//*****************************************************************************
am_devices_spiflash_t g_sSpiFlash =
{
    .ui32IOMModule = AM_BSP_FLASH_IOM,
    .ui32ChipSelect = AM_BSP_FLASH_CS,
};

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Configure GPIOs for communicating with a SPI flash
//
//*****************************************************************************
static void
configure_spiflash_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_FLASH_IOM);

    //
    // Enable the chip-select and data-ready pins for the SPI FLASH
    //
    am_bsp_pin_enable(FLASH_CS);
}

static int ext_flash_erase_sector(uint32_t ui32DstAddr)
{
    am_devices_spiflash_sector_erase(ui32DstAddr);
    return 0;
}

static int
ext_flash_init(void)
{
    //storage in external flash

#if defined(AM_PART_APOLLO2)
    //
    // Power on SPI
    //
    am_hal_iom_pwrctrl_enable(AM_BSP_FLASH_IOM);
#endif

    //
    // initialize spi interface with external flash
    //
    am_hal_iom_config(AM_BSP_FLASH_IOM, &g_sIOMConfig);

#if defined(AM_PART_APOLLO2)
    //
    // Save the configuration
    //
    am_hal_iom_power_off_save(AM_BSP_FLASH_IOM);
#endif
    //
    // configure pins for iom interface
    //
    configure_spiflash_pins();

    //
    // Initialize the spiflash driver with the IOM information for the second
    // flash device.
    //
    am_devices_spiflash_init(&g_sSpiFlash);
    return 0;
}

static int
ext_flash_enable(void)
{
    //
    // Turn on the IOM for this operation.
    //
#if defined(AM_PART_APOLLO2)
    am_hal_iom_power_on_restore(AM_BSP_FLASH_IOM);
#endif
    am_hal_iom_enable(AM_BSP_FLASH_IOM);
    am_bsp_iom_spi_pins_enable(AM_BSP_FLASH_IOM);
    return 0;
}

static int
ext_flash_disable(void)
{
    //
    // Disable IOM1 SPI pins and turn off the IOM for this operation.
    //
    am_bsp_iom_spi_pins_disable(AM_BSP_FLASH_IOM);
    am_hal_iom_disable(AM_BSP_FLASH_IOM);
#if defined(AM_PART_APOLLO2)
    am_hal_iom_power_off_save(AM_BSP_FLASH_IOM);
#endif
    return 0;
}

static int
ext_flash_deinit(void)
{
    return 0;
}

static int
ext_flash_write_page(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length)
{
    am_devices_spiflash_write((uint8_t *)pSrc, ui32DestAddr, ui32Length);
    return 0;
}

static int
ext_flash_read_page(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length)
{
    am_devices_spiflash_read((uint8_t *)ui32DestAddr, (uint32_t)pSrc, ui32Length);
    return 0;
}

am_multiboot_flash_info_t g_extFlash =
{
    .flashPageSize      = AM_DEVICES_SPIFLASH_PAGE_SIZE,
    .flashSectorSize    = AM_DEVICES_SPIFLASH_SECTOR_SIZE,
    .flash_init         = ext_flash_init,
    .flash_deinit       = ext_flash_deinit,
    .flash_enable       = ext_flash_enable,
    .flash_disable      = ext_flash_disable,
    .flash_read_page    = ext_flash_read_page,
    .flash_write_page   = ext_flash_write_page,
    .flash_erase_sector = ext_flash_erase_sector,
};

