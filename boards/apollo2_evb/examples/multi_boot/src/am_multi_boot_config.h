//*****************************************************************************
//
//! @file am_multi_boot_config.h
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

#ifndef AM_MULTI_BOOT_CONFIG_H
#define AM_MULTI_BOOT_CONFIG_H


//*****************************************************************************
//
// Run with flag page.
//
//*****************************************************************************
#define USE_FLAG_PAGE                       1

//*****************************************************************************
//
// Support for external SPI NOR Flash.
//
//*****************************************************************************
// Note: this will increase the bootloader size by about 8.5KB.
// Note: this will increase the SRAM size by about 60KB.
#define USE_EXTERNAL_FLASH                  0

//*****************************************************************************
//
// Location of the flag page.
//
//*****************************************************************************
#define FLAG_PAGE_LOCATION                 0x00006000

//*****************************************************************************
//
// Max Size of Bootloader.
//
//*****************************************************************************
// The value here must match (at least) with the ROLength restriction imposed at
// bootloader linker configuration
#if USE_EXTERNAL_FLASH == 1
#define MAX_BOOTLOADER_SIZE                0x00005000
#else
#define MAX_BOOTLOADER_SIZE                0x00003000
#endif //USE_EXTERNAL_FLASH == 1

// The value here must match (at least) with the RWLength restriction imposed at
// bootloader linker configuration
#if USE_EXTERNAL_FLASH == 1
#define MAX_SRAM_USED                      0x00016000
#else
#define MAX_SRAM_USED                      0x00006000
#endif

//*****************************************************************************
//
// Default Override Pin mapped to Button1
//
//*****************************************************************************
#define DEFAULT_OVERRIDE_GPIO              AM_BSP_GPIO_BUTTON1

//*****************************************************************************
//
// I2C Address to use
//
//*****************************************************************************
#define I2C_SLAVE_ADDR                     0x10

//*****************************************************************************
// Definitions below are don't care for multiboot-core
//*****************************************************************************
//*****************************************************************************
//
// Multiboot mode supported.
//
//*****************************************************************************
#define AM_MULTIBOOT_SUPPORT_IOS // enables support for boot over IOS (SPI/I2C)
#define AM_MULTIBOOT_SUPPORT_UART // enables support for boot over UART
#define AM_MULTIBOOT_SUPPORT_OTA // enables support for OTA upgrade
//*****************************************************************************
//
// Location of the OTA_POINTER
// This should ideally be a separate page.
// However, currently set to an offset within the flag page, just to avoid
// wasting a flash page.
//
//*****************************************************************************
//#define OTA_POINTER_LOCATION               0x16000
#define OTA_POINTER_LOCATION               (FLAG_PAGE_LOCATION + 256)

//
// IOS interrupt pin to use
//
#define MULTIBOOT_IOS_INTERRUPT_PIN        4

// Time to wait for host to initiate download over IOS before switching to UART
#define WAIT_IOS_BOOT_MS                    200
#define WAIT_IOS_BOOT_SYSTICK               ((AM_HAL_CLKGEN_FREQ_MAX_HZ/1000)*WAIT_IOS_BOOT_MS)

#define MULTIBOOT_DETECT_BAUDRATE
#define MULTIBOOT_UART_BAUDRATE             115200

#endif // AM_MULTI_BOOT_CONFIG_H
