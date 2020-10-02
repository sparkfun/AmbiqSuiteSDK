//*****************************************************************************
//
//! @file amota_profile_config.h
//!
//! @brief Global bootloader information.
//!
//! This is a bootloader program that supports flash programming over UART,
//! SPI, and I2C. The correct protocol is selected automatically at boot time.
//!
//! SWO is configured in 1M baud, 8-n-1 mode.
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

#ifndef AMOTA_PROFILE_CONFIG_H
#define AMOTA_PROFILE_CONFIG_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_bootloader.h"

//*****************************************************************************
//
// Run with flag page.
//
//*****************************************************************************
#define USE_FLAG_PAGE                       1

//
// Set to 1 to use the last flash page as the flash flag area.
// (Default 0)
//
#define USE_LAST_PAGE_FOR_FLAG              0

//*****************************************************************************
//
// Location of the flag page.
//
//*****************************************************************************
#define FLAG_PAGE_LOCATION                  0x00006000

//*****************************************************************************
//
// Location of the new image storage in internal flash.
//
//*****************************************************************************
//
// Note: Internal flash area to be used for OTA temporary storage
// The address must be aligned to flash page
// This should be customized to the desired memory map of the design
//
#define AMOTA_INT_FLASH_OTA_ADDRESS         0x00030000

//
// User specified maximum size of OTA storage area.
// Make sure the size is flash page multiple
// (Default value is determined based on rest of flash from the start)
//
#if (USE_LAST_PAGE_FOR_FLAG == 1)
#define AMOTA_INT_FLASH_OTA_MAX_SIZE        (AM_HAL_FLASH_LARGEST_VALID_ADDR - AMOTA_INT_FLASH_OTA_ADDRESS + 1 - AM_HAL_FLASH_PAGE_SIZE)
#else
#define AMOTA_INT_FLASH_OTA_MAX_SIZE        (AM_HAL_FLASH_LARGEST_VALID_ADDR - AMOTA_INT_FLASH_OTA_ADDRESS + 1)
#endif //#if (USE_LAST_PAGE_FOR_FLAG == 1)


// OTA Descriptor address
// For this implementation, we are setting OTA_POINTER in the flag page, following the
// image info to avoid wasting another flash page
// This means that care needs to be taken to preserve the existing contents
// of the page when updating the OTA descriptor
#define OTA_POINTER_LOCATION                (FLAG_PAGE_LOCATION + 256)


//*****************************************************************************
//
// Support for External Flash
//
//*****************************************************************************
#define AMOTAS_SUPPORT_EXT_FLASH            0

#if (AMOTAS_SUPPORT_EXT_FLASH == 1)
#include "extflash.h"
//
// Fixed external SPI Flash storage start address to be defined by user.
// Currently defined arbitrarily for a 256K size flash
//
#define AMOTA_EXT_FLASH_OTA_ADDRESS         0
#define AMOTA_EXT_FLASH_OTA_MAX_SIZE        0x40000
#endif


#endif // AMOTA_PROFILE_CONFIG_H
