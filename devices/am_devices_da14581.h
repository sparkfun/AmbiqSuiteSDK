//*****************************************************************************
//
//! @file am_devices_da14581.h
//!
//! @brief Support functions for the Dialog Semiconductor DA14581 BTLE radio.
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
#ifndef AM_DEVICES_DA14581_H
#define AM_DEVICES_DA14581_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define AM_DEVICES_DA14581_UART_MODE         (0)
#define AM_DEVICES_DA14581_SPI_MODE          (1)

#define AM_DEVICES_DA14581_SPI_XPORT_CTS     (0x06)
#define AM_DEVICES_DA14581_SPI_XPORT_NOT_CTS (0x07)

//*****************************************************************************
//
// DA14581 device structure
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
    // GPIO # for DA14581 DREADY signal
    //
    uint32_t ui32DREADY;
}
am_devices_da14581_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_devices_da14581_mac_set(const uint8_t *pui8MacAddress);
extern bool am_devices_da14581_uart_boot(const uint8_t *pui8BinData,
                                         uint32_t ui32NumBytes,
                                         uint32_t ui32UartModule);
#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_DA14581_H
