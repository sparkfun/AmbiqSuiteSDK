//*****************************************************************************
//
//! @file am_devices_em9304.h
//!
//! @brief Support functions for the EM Micro EM9304 BTLE radio.
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
#ifndef AM_DEVICES_EM9304_H
#define AM_DEVICES_EM9304_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// EM9304 device structure
//
//*****************************************************************************
typedef struct
{
    //
    // MODE UART vs IOM SPI
    //
    uint32_t ui32Mode;

    //
    // IOM Module #
    //
    uint32_t ui32IOMModule;

    //
    // IOM Chip Select NOTE: this driver uses GPIO for chip selects
    //
    uint32_t ui32IOMChipSelect;

    //
    // GPIO # for EM9304 DREADY signal
    //
    uint32_t ui32DREADY;
}
am_devices_em9304_t;

extern const am_devices_em9304_t g_sEm9304;
#if defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)
extern const am_hal_iom_config_t g_sEm9304IOMConfigSPI;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern bool am_devices_em9304_mac_set(const uint8_t *pui8MacAddress);
extern uint32_t am_devices_em9304_block_read(const am_devices_em9304_t *psDevice,
                                             uint32_t *pui32Values,
                                             uint32_t ui32NumBytes);
extern void am_devices_em9304_block_write(const am_devices_em9304_t *psDevice,
                                              uint8_t type,
                                              uint8_t *pui8Values,
                                              uint32_t ui32NumBytes);

extern void am_devices_em9304_spi_init(uint32_t ui32Module, const am_hal_iom_config_t *psIomConfig);
extern uint8_t am_devices_em9304_tx_starts(const am_devices_em9304_t *psDevice);
extern void am_devices_em9304_tx_ends(void);
extern void am_devices_em9304_config_pins(void);
extern void am_devices_em9304_spi_sleep(uint32_t ui32Module);
extern void am_devices_em9304_spi_awake(uint32_t ui32Module);
extern void am_devices_em9304_enable_interrupt(void);
extern void am_devices_em9304_disable_interrupt(void);
#endif // defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)

#if (defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P))

typedef enum
{
    AM_DEVICES_EM9304_STATUS_SUCCESS,
    AM_DEVICES_EM9304_STATUS_ERROR
} am_devices_em9304_status_t;

typedef struct
{
    uint32_t ui32ClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
} am_devices_em9304_config_t;


#define AM_DEVICES_EM9304_MAX_DEVICE_NUM    1

extern uint32_t am_devices_em9304_init(uint32_t ui32Module, am_devices_em9304_config_t *pDevConfig, void **ppHandle, void **ppIomHandle);
extern uint32_t am_devices_em9304_term(void *pHandle);


#endif // defined(AM_PART_APOLLO3)

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_EM9304_H
