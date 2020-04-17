//*****************************************************************************
//
//! @file spi_boot_host.c
//!
//! @brief An example to drive the IO Slave on a second board.
//!
//! This example acts as the boot host for spi_boot and multi_boot on Apollo
//! and Apollo2 MCUs. It will deliver a predefined firmware image to a boot
//! slave over a SPI protocol. The purpose of this demo is to show how a host
//! processor might store, load, and update the firmware on an Apollo or
//! Apollo2 device that is connected as a slave.
//!
//! Please see the multi_boot README.txt for more details on how to run the
//! examples.
//!
//! @verbatim
//! PIN fly lead connections assumed by multi_boot:
//!     HOST                                    SLAVE (multi_boot target)
//!     --------                                --------
//!     GPIO[2]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[4]  OVERRIDE pin   (host to slave) GPIO[18] Override pin or n/c
//!     GPIO[5]  IOM0 SPI CLK/I2C SCL           GPIO[0]  IOS SPI SCK/I2C SCL
//!     GPIO[6]  IOM0 SPI MISO/I2C SDA          GPIO[1]  IOS SPI MISO/I2C SDA
//!     GPIO[7]  IOM0 SPI MOSI                  GPIO[2]  IOS SPI MOSI
//!     GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
//!     GPIO[17] Slave reset (host to slave)    Reset Pin or n/c
//!     GND                                     GND
//! Reset and Override pin connections from Host are optional
//! Keeping Button1 pressed on target has same effect as host driving override
//! @endverbatim
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
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Download image details.
//
// This header file represents the binary image that the boot host will try to
// load onto the slave device.
//
// By default, the demo image used is the same as the host device.  That is,
// the slave device to which the demo image is downloaded is assumed to be
// the same device as the host.  If the 2 devices are not the same, the
// appropriate image to be downloaded to the slave can be selected here.
//
//*****************************************************************************
//#define USE_APOLLO1_DEMO    1
//#define USE_APOLLO2_DEMO    1

#if defined(USE_APOLLO1_DEMO)
#include "apollo_boot_demo.h"
#elif defined(USE_APOLLO2_DEMO)
#include "apollo2_boot_demo.h"
#elif defined(AM_PART_APOLLO)
#include "apollo_boot_demo.h"
#elif defined(AM_PART_APOLLO2)
#include "apollo2_boot_demo.h"
#endif

// This assumes Host is running on same type of board as target.
// If that is not true, this definition needs to be adjusted to match the
// desired pin on target board
#define TARGET_BOARD_OVERRIDE_PIN          AM_BSP_GPIO_BUTTON1
// Slave interrupt pin is connected here
#define BOOTLOADER_HANDSHAKE_PIN           2
// This pin is connected to RESET pin of slave
#define DRIVE_SLAVE_RESET_PIN              17
// This pin is connected to the 'Override' pin of slave
#define DRIVE_SLAVE_OVERRIDE_PIN           4

//*****************************************************************************
//
// Boot Commands.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK_CMD               0x00000000
#define AM_BOOTLOADER_NAK_CMD               0x00000001
#define AM_BOOTLOADER_NEW_IMAGE             0x00000002
#define AM_BOOTLOADER_NEW_PACKET            0x00000003
#define AM_BOOTLOADER_RESET                 0x00000004
#define AM_BOOTLOADER_SET_OVERRIDE_CMD      0x00000005
#define AM_BOOTLOADER_BL_VERSION_CMD        0x00000006
#define AM_BOOTLOADER_FW_VERSION_CMD        0x00000007

//*****************************************************************************
//
// Slave messages.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK                   0x00000000
#define AM_BOOTLOADER_NAK                   0x00000001
#define AM_BOOTLOADER_READY                 0x00000002
#define AM_BOOTLOADER_IMAGE_COMPLETE        0x00000003
#define AM_BOOTLOADER_BAD_CRC               0x00000004
#define AM_BOOTLOADER_ERROR                 0x00000005
#define AM_BOOTLOADER_BL_VERSION            0x00000006
#define AM_BOOTLOADER_FW_VERSION            0x00000007

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
am_hal_iom_buffer(256) g_psTxBuffer;
am_hal_iom_buffer(256) g_psRxBuffer;

//*****************************************************************************
//
// AES information
//
//*****************************************************************************
uint32_t pui32ExpandedKey[60];

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_100KHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 4,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Configure GPIOs for this example
//
//*****************************************************************************
void
configure_pins(void)
{
    //
    // Configure I/O Master 0 as SPI
    //
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCK);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0MISO);
    am_hal_gpio_pin_config(7, AM_HAL_PIN_7_M0MOSI);
    am_hal_gpio_pin_config(11, AM_HAL_PIN_11_M0nCE0);


    //
    // Configure the I/O Slave interrupt pin
    //
    am_hal_gpio_pin_config(BOOTLOADER_HANDSHAKE_PIN, AM_HAL_PIN_INPUT | AM_HAL_GPIO_PULLUP);
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);
}

//*****************************************************************************
//
// Reset the slave device and force it into boot mode.
//
//*****************************************************************************
void
start_boot_mode(void)
{
    //
    // Drive RESET low.
    //
    am_hal_gpio_out_bit_clear(DRIVE_SLAVE_RESET_PIN);
    am_hal_gpio_pin_config(DRIVE_SLAVE_RESET_PIN, AM_HAL_PIN_OUTPUT);

    //
    // Drive the override pin low to force the slave into boot mode.
    //
    am_hal_gpio_out_bit_clear(DRIVE_SLAVE_OVERRIDE_PIN);
    am_hal_gpio_pin_config(DRIVE_SLAVE_OVERRIDE_PIN, AM_HAL_PIN_OUTPUT);

    //
    // Short delay.
    //
    am_util_delay_us(5);

    //
    // Release RESET.
    //
    am_hal_gpio_out_bit_set(DRIVE_SLAVE_RESET_PIN);

    //
    // Wait for the slave to Set the handshake pin
    //
    while ( !am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );
}

//*****************************************************************************
//
// Send the commands to start a new boot download.
//
//*****************************************************************************
void
start_new_image(void)
{
    //
    // Wait for the slave to send the ready signal
    //
    while ( am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );
    //
    // Make sure the override pin is high so the slave will reboot into
    // application mode when our boot procedure is complete.
    //
    am_hal_gpio_out_bit_set(DRIVE_SLAVE_OVERRIDE_PIN);
    // Clear any interrupts that may have happened while Slave is coming up
    am_hal_iom_int_clear(0, 0xFFFFFFFF);
    am_hal_iom_spi_read(0, 0, g_psRxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x0));

    //
    // ACK the ready signal to have slave pull the interrupt line high.
    //
    g_psTxBuffer.words[0] = AM_BOOTLOADER_ACK_CMD;
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
    //
    // Wait for the slave to read the ACK
    //
    while ( !am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );

    //
    // Write the image parameters to the SPI FIFO
    //
    g_psTxBuffer.words[0] = IMAGE_LINK_ADDRESS;
    g_psTxBuffer.words[1] = IMAGE_SIZE;
    g_psTxBuffer.words[2] = IMAGE_CRC;

    //
    // Send the image parameters to the slave.
    //
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 12, AM_HAL_IOM_OFFSET(0x84));

    //
    // Finish out the image start routine with the "New Image" packet.
    //
    g_psTxBuffer.words[0] = AM_BOOTLOADER_NEW_IMAGE;
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
}

//*****************************************************************************
//
// Set override pin.
//
//*****************************************************************************
void
override_pin_set(uint32_t ui32OverridePin, uint32_t ui32OverridePolarity)
{
    //
    // Wait for the slave to send the ready signal
    //
    while ( am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );
    am_hal_iom_spi_read(0, 0, g_psRxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x0));

    //
    // ACK the ready signal to have slave pull the interrupt line high.
    //
    g_psTxBuffer.words[0] = AM_BOOTLOADER_ACK_CMD;
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
    //
    // Wait for the slave to read the ACK
    //
    while ( !am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );

    //
    // Write the image parameters to the SPI FIFO
    //
    g_psTxBuffer.words[0] = ui32OverridePin;
    g_psTxBuffer.words[1] = ui32OverridePolarity;

    //
    // Send the image parameters to the slave.
    //
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 8, AM_HAL_IOM_OFFSET(0x84));

    //
    // Finish out the image start routine with the "New Image" packet.
    //
    g_psTxBuffer.words[0] = AM_BOOTLOADER_SET_OVERRIDE_CMD;
    am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
}

//*****************************************************************************
//
// Send the actual firmware image over to the boot slave.
//
//*****************************************************************************
void
transfer_image(void)
{
    uint32_t ui32BytesRemaining;
    uint32_t ui32TransferSize;
    uint32_t ui32Offset;
    uint32_t i;

    //
    // Send the firmware image across.
    //
    ui32BytesRemaining = IMAGE_SIZE;
    ui32Offset = 0;

    while ( ui32BytesRemaining )
    {
        //
        // Wait for another ready signal.
        //
        while ( am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );
        am_hal_iom_spi_read(0, 0, g_psRxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x0));
        //
        // ACK the ready signal to have slave pull the interrupt line high.
        //
        g_psTxBuffer.words[0] = AM_BOOTLOADER_ACK_CMD;
        am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
        //
        // Wait for the slave to read the ACK
        //
        while ( !am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );

        //
        // We can't transfer more than a few bytes at a time. Limit the
        // transaction to 112 bytes max.
        //
        ui32TransferSize = ui32BytesRemaining > 112 ? 112 : ui32BytesRemaining;

        //
        // Start the packet with the packet length.
        //
        g_psTxBuffer.words[0] = ui32TransferSize;

        //
        // Fill in the packet contents.
        //
        for ( i = 0; i < ui32TransferSize; i++ )
        {
            g_psTxBuffer.bytes[4 + i] = IMAGE_ARRAY[ui32Offset + i];
        }

        //
        // Send the data over to the slave.
        //
        am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, ui32TransferSize + 4,
                             AM_HAL_IOM_OFFSET(0x84));

        //
        // Finish with the "New Packet" boot command.
        //
        g_psTxBuffer.words[0] = AM_BOOTLOADER_NEW_PACKET;
        am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));

        //
        // Update the loop variables.
        //
        ui32BytesRemaining -= ui32TransferSize;
        ui32Offset += ui32TransferSize;
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Setup the pins for IO Master Example.
    //
    configure_pins();

    //
    // Initialize IOM 0 in SPI mode at 100KHz
    //
#ifndef AM_PART_APOLLO
    am_hal_iom_pwrctrl_enable(0);
#endif
    am_hal_iom_config(0, &g_sIOMConfig);
    //
    // Turn on the IOM for this operation.
    //
    am_bsp_iom_enable(0);

    //
    // Force the slave into boot mode.
    //
    start_boot_mode();

    //
    // Wait for the 'READY' from the boot slave, and then send the packet
    // information.
    //
    start_new_image();

    //
    // Change the override pin to correspond to a button on the Apollo EVK
    //
    override_pin_set(TARGET_BOARD_OVERRIDE_PIN, 0);

    //
    // Wait for another 'READY', and send the actual image across.
    //
    transfer_image();

    //
    // At this point, the slave should send back a either 'CRC OK' or some sort
    // of error. If the CRC was good, we should tell the slave to reset itself
    // and run the new image.
    //
    while ( am_hal_gpio_input_bit_read(BOOTLOADER_HANDSHAKE_PIN) );
    am_hal_iom_spi_read(0, 0, g_psRxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x0));

    if ( g_psRxBuffer.words[0] == AM_BOOTLOADER_IMAGE_COMPLETE )
    {
        //
        // If the CRC is correct, send a RESET command.
        //
        g_psTxBuffer.words[0] = AM_BOOTLOADER_RESET;
        am_hal_iom_spi_write(0, 0, g_psTxBuffer.words, 4, AM_HAL_IOM_OFFSET(0x80));
    }

    //
    // Loop forever.
    //
    while (1)
    {
    }
}
