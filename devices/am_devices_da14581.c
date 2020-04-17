//*****************************************************************************
//
//! @file am_devices_da14581.c
//!
//! @brief Support functions for the Dialog Semiconductor DA14581 BTLE radio.
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

#include <stdint.h>
#include <stdbool.h>
#include "am_bsp.h"
#include "am_devices_da14581.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define DIALOG_BOOT_STX                     0x02
#define DIALOG_BOOT_SOH                     0x01
#define DIALOG_BOOT_ACK                     0x06
#define DIALOG_BOOT_NACK                    0x15

//*****************************************************************************
//
// BLE MAC address for the Dialog radio.
//
//*****************************************************************************
static uint8_t g_BLEMacAddress[6] =
{0x01, 0x00, 0x00, 0xCA, 0xEA, 0x80};

//*****************************************************************************
//
// Location in the Diaog binary where the MAC address is stored.
//
//*****************************************************************************
static const uint32_t g_ui32MacIndex = 1000;

//*****************************************************************************
//
// Sets the MAC address to send to the dialog radio on the next boot up.
//
//*****************************************************************************
void
am_devices_da14581_mac_set(const uint8_t *pui8MacAddress)
{
    uint32_t i;

    //
    // Copy the 6-byte MAC address into our global variable.
    //
    for ( i = 0; i < 6; i++ )
    {
        g_BLEMacAddress[i] = *pui8MacAddress++;
    }
}

//*****************************************************************************
//
//! @brief Runs a UART based boot sequence for a Dialog radio device.
//!
//! @param pui8BinData - pointer to an array of bytes containing the firmware
//! for the DA14581
//!
//! @param ui32NumBytes - length of the DA14581 firmware image.
//!
//! This function allows the Ambiq device to program a "blank" DA14581 device
//! on startup. It will handle all of the necessary UART negotiation for the
//! Dialog boot procedure, and will verify that the CRC value for the
//! downloaded firmware image is correct.
//!
//! @return true if successful.
//
//*****************************************************************************
bool
am_devices_da14581_uart_boot(const uint8_t *pui8BinData, uint32_t ui32NumBytes,
                             uint32_t ui32UartModule)
{
    uint32_t ui32Index;
    uint8_t ui8CRCValue;
    uint8_t ui8TxData;
    char ui8RxChar;

    //
    // Poll the RX lines until we get some indication that the dialog radio is
    // present and ready to receive data.
    //
    do
    {
        am_hal_uart_char_receive_polled(ui32UartModule, &ui8RxChar);
    }
    while (ui8RxChar != DIALOG_BOOT_STX);

    //
    // Send the Start-of-Header signal and the length of the data download.
    //
    am_hal_uart_char_transmit_polled(ui32UartModule, DIALOG_BOOT_SOH);
    am_hal_uart_char_transmit_polled(ui32UartModule, ui32NumBytes & 0xFF);
    am_hal_uart_char_transmit_polled(ui32UartModule, (ui32NumBytes & 0xFF00) >> 8);

    //
    // Poll for the 'ACK' from the dialog device that signifies that the header
    // was recieved correctly.
    //
    do
    {
        am_hal_uart_char_receive_polled(ui32UartModule, &ui8RxChar);
    }
    while (ui8RxChar != DIALOG_BOOT_ACK);

    //
    // Initialize the CRC value to zero.
    //
    ui8CRCValue = 0;

    //
    // Send the binary image over to the dialog device one byte at a time,
    // keeping track of the CRC as we go.
    //
    for (ui32Index = 0; ui32Index < ui32NumBytes; ui32Index++)
    {
        if ((ui32Index >= g_ui32MacIndex) && (ui32Index < g_ui32MacIndex + 6))
        {
            ui8TxData = g_BLEMacAddress[ui32Index - g_ui32MacIndex];
        }
        else
        {
            ui8TxData = pui8BinData[ui32Index];
        }

        ui8CRCValue ^= ui8TxData;
        am_hal_uart_char_transmit_polled(ui32UartModule, ui8TxData);
    }

    //
    // The Dialog device should respond back with a CRC value at the end of the
    // programming cycle. We should check here to make sure that they got the
    // same CRC result that we did. If it doesn't match, return with an error.
    //
    am_hal_uart_char_receive_polled(ui32UartModule, &ui8RxChar);

    if ( ui8RxChar != ui8CRCValue )
    {
        return 1;
    }

    //
    // If all is well, send the final 'ACK' to tell the dialog device that its
    // new image is correct. After this point, the dialog device should start
    // running the downloaded firmware.
    //
    am_hal_uart_char_transmit_polled(ui32UartModule, DIALOG_BOOT_ACK);

    //
    // Wait until the FIFO is actually empty and the UART is no-longer busy.
    //
    while (!AM_BFR(UART, FR, TXFE) || AM_BFR(UART, FR, BUSY));

    return 0;
}

