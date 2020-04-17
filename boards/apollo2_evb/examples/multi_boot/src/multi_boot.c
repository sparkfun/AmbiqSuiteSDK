//*****************************************************************************
//
//! @file multi_boot.c
//!
//! @brief Bootloader program accepting multiple host protocols.
//!
//! Multiboot is a bootloader program that supports flash programming over UART,
//! SPI, and I2C. The correct protocol is selected automatically at boot time.
//! The messaging is expected to follow little-endian format, which is native to
//! the Cortex M4 used in Apollo and Apollo2.
//!
//! If a valid image is already present on the target device, Multiboot will run
//! that image.  Otherwise it will wait for a new image to be downloaded from
//! the host.  A new image download can be forced on the target by using the
//! "override" capability via one of the pins.
//!
//! Running this example requires 2 EVBs - one EVB to run multi_boot, the second
//! to run a host example such as spi_boot_host or i2c_boot_host. The two EVBs
//! are generally fly wired together as shown below.
//!
//! The most straightforward method of running the demonstration is to use two
//! EVBs of the same type (i.e. 2 Apollo EVBs or 2 Apollo2 EVBs) as both the
//! host and the target.  Then the pins on the two EVBs are connected as shown
//! in the chart including the optional reset and override pins.  With these
//! connections, the host controls everything and no user intervention is
//! required other than the press the reset button on the host to initiate the
//! process.
//!
//! The host downloads an executable (target) image to the slave that will run
//! on the target.  By default that image is binary_counter, which is obvious
//! to see running on the slave.
//!
//! In the default scenario (two boards of the same type), the image downloaded
//! by the host is compatible with the host board type, so it will run without
//! modification on the target.  In the case of the host device being different
//! from the target, the image downloaded from the host must be modified to be
//! compatible with the target (requires rebuilding the host example).
//!
//! The EVB Button1 (usually labelled BTN2 on the EVB silkscreen) is the manual
//! override which can be used to force downloading of a new image even if a
//! valid image already exists on the target. The host examples use the same
//! "override" pin signal when downloading a new image.
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
//!     GPIO[17] Slave reset (host to slave)    Reset Pin (NRST) or n/c
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_multi_boot_config.h"
#include "am_multi_boot.h"

#if USE_EXTERNAL_FLASH == 1
#include "extflash.h"
#endif //USE_EXTERNAL_FLASH == 1

#ifdef AM_MULTIBOOT_SUPPORT_IOS
//*****************************************************************************
//
// Global variable to let the main loop know whether the IOS interface has been
// used yet.
//
//*****************************************************************************
volatile bool g_bIOSActive = false;

//*****************************************************************************
//
// IO Slave Register Access ISR.
//
//*****************************************************************************
void
am_ioslave_acc_isr(void)
{
    //
    // Make sure the main loop knows that the IOS is currently in use.
    //
    g_bIOSActive = true;

    am_multiboot_ios_acc_isr_handler();
}
#endif

#ifdef AM_MULTIBOOT_SUPPORT_UART
#ifdef MULTIBOOT_DETECT_BAUDRATE
volatile bool g_bUartConfigured = false;
//*****************************************************************************
//
// Setting up a GPIO ISR for the UART autobaud feature.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint32_t ui32BaudRate;

    ui32BaudRate = am_multiboot_uart_detect_baudrate(AM_BSP_GPIO_BOOTLOADER_UART_RX);

    //
    // Now that we have a baud rate, we can configure our UART.
    //
    am_multiboot_setup_serial(AM_BSP_UART_BOOTLOADER_INST, ui32BaudRate);

    //
    // Send a '0x55' to give the boot host some indication that we have the
    // correct baud rate and to let it know that our UART is ready for further
    // traffic.
    //
    am_hal_uart_char_transmit_polled(AM_BSP_UART_BOOTLOADER_INST, 0x55);
    g_bUartConfigured = true;
}
#endif
//*****************************************************************************
//
// UART ISR
//
//*****************************************************************************
void
#if (AM_BSP_UART_BOOTLOADER_INST == 0)
am_uart_isr(void)
#elif (AM_BSP_UART_BOOTLOADER_INST == 1)
am_uart1_isr(void)
#endif
{
    am_multiboot_uart_isr_handler(AM_BSP_UART_BOOTLOADER_INST);
}
#endif

// Need temporary buf equal to one flash sector size (larger of int or ext, if ext flash is supported)
// We accumulate data in this buffer and perform Writes only on page boundaries in
// multiple of page lentghs
#if (USE_EXTERNAL_FLASH == 1) && (AM_DEVICES_SPIFLASH_SECTOR_SIZE > AM_HAL_FLASH_PAGE_SIZE)
#define TEMP_BUFSIZE     AM_DEVICES_SPIFLASH_SECTOR_SIZE
#else
#define TEMP_BUFSIZE     AM_HAL_FLASH_PAGE_SIZE
#endif


uint32_t sramTempBuf[TEMP_BUFSIZE / 4];

#ifdef AM_MULTIBOOT_SUPPORT_OTA
// Invalidate the OTA - so that it is not processed again for next boot
void invalidate_ota(am_multiboot_ota_t *pOtaInfo)
{
    uint32_t tempZero;
    uint32_t otaPtrVal = *((uint32_t *)OTA_POINTER_LOCATION);
    // CAUTION: We can reprogram a bit in flash to 0 only once...so make sure we do not re-clear bits
    tempZero = ~otaPtrVal;
    // clear the value for subsequent boots
    am_bootloader_write_flash_within_page(OTA_POINTER_LOCATION, &tempZero, 1);
}
#endif

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    am_bootloader_image_t *pBootImage;
    bool bOverride = false;
    bool bBootFromFlash = false;
#ifdef AM_MULTIBOOT_SUPPORT_OTA
    uint32_t otaPtrVal = *((uint32_t *)OTA_POINTER_LOCATION);
#endif

#ifdef AM_MULTIBOOT_SUPPORT_IOS
    //
    // Set the (active LOW) interrupt pin so the host knows we don't have a
    // message to send yet.
    //
    am_hal_gpio_out_bit_set(MULTIBOOT_IOS_INTERRUPT_PIN);
    am_hal_gpio_pin_config(MULTIBOOT_IOS_INTERRUPT_PIN, AM_HAL_PIN_OUTPUT);
#endif

    bBootFromFlash = am_multiboot_check_boot_from_flash(&bOverride, &pBootImage);
    if (!bOverride)
    {
#ifdef AM_MULTIBOOT_SUPPORT_OTA
        // Check if OTA available
        if ((otaPtrVal != 0xFFFFFFFF) && (otaPtrVal != 0))
        {
            //
            // Configure the board for low power.
            //
            am_bsp_low_power_init();
            //
            // If we get here, we're going to try to download a new image from a host
            // processor. Speed up the clocks and start turning on peripherals.
            //
            am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);
            // function below does not return in case of success

#if USE_EXTERNAL_FLASH == 1
            if (am_multiboot_ota_handler((void *)otaPtrVal, sramTempBuf, sizeof(sramTempBuf), invalidate_ota, &g_extFlash) == false)
#else
            if (am_multiboot_ota_handler((void *)otaPtrVal, sramTempBuf, sizeof(sramTempBuf), invalidate_ota, NULL) == false)
#endif //USE_EXTERNAL_FLASH == 1
            {
                if (bBootFromFlash)
                {
                    // We want to run the flash image with clean slate...
                    // So doing a POI here, and the image will be run in the next boot
                    //
                    // Perform a software reset.
                    //
                    am_hal_reset_poi();
                }
            }
        }
#endif
        if (bBootFromFlash)
        {
#ifdef AM_MULTIBOOT_SUPPORT_IOS
            //
            // If everything looks good, disable the interrupt pin and run.
            //
            am_hal_gpio_pin_config(MULTIBOOT_IOS_INTERRUPT_PIN, AM_HAL_PIN_DISABLE);
#endif
            am_bootloader_image_run(pBootImage);
        }
    }
    //
    // If we get here, we're going to try to download a new image from a host
    // processor. Speed up the clocks and start turning on peripherals.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);
    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // Provide temp buffer for multiboot to operate on
    //
    am_multiboot_init(sramTempBuf, sizeof(sramTempBuf));

#ifdef AM_MULTIBOOT_SUPPORT_IOS
    //
    // Start systick to measure time for the IOS timeout.
    //
    am_hal_systick_load(0x00FFFFFF);
    am_hal_systick_start();

    //
    // Enable the IOS. Choose the correct protocol based on the state of pin 0.
    //
    am_multiboot_setup_ios_interface(MULTIBOOT_IOS_INTERRUPT_PIN);

    //
    // Wait for a few milliseconds to see if anyone will send us data.
    //
    while ( g_bIOSActive
#ifdef AM_MULTIBOOT_SUPPORT_UART
        || am_hal_systick_count() > (0xFFFFFF - WAIT_IOS_BOOT_SYSTICK)
#endif
        )
    {
        //
        // Delay to avoid polling peripheral registers so frequently.
        //
        am_util_delay_ms(1);
    }

    //
    // If we didn't get any IOS packets, we'll move on to the UART option.
    //
    am_multiboot_cleanup_ios_interface();
    am_hal_gpio_pin_config(MULTIBOOT_IOS_INTERRUPT_PIN, AM_HAL_PIN_DISABLE);
#endif

#ifdef AM_MULTIBOOT_SUPPORT_UART
#ifdef MULTIBOOT_DETECT_BAUDRATE
    //
    // Re-Start systick to measure time for autobaud and for the IOS timeout.
    //
    am_hal_systick_stop();
    am_hal_systick_load(0x00FFFFFF);
    am_hal_systick_start();

    //
    // Configure our RX pin as a GPIO input with a falling edge interrupt.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_BOOTLOADER_UART_RX, AM_HAL_GPIO_INPUT);
    am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_BOOTLOADER_UART_RX, AM_HAL_GPIO_FALLING);

    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BOOTLOADER_UART_RX));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BOOTLOADER_UART_RX));
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);

    //
    // Enable interrupts so we can receive messages from the boot host.
    //
    am_hal_interrupt_master_enable();
    while (1)
    {
        //
        // Disable interrupt while we decide whether we're going to sleep.
        //
        uint32_t ui32IntStatus = am_hal_interrupt_master_disable();

        if (!g_bUartConfigured)
        {
            // Wait for Baud rate detection
            am_hal_sysctrl_sleep(false);
            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
        }
        else
        {
            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
            break;
        }
    }
    // ISR has already configured the UART by now
    am_hal_gpio_int_disable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BOOTLOADER_UART_RX));
    am_hal_interrupt_disable(AM_HAL_INTERRUPT_GPIO);
#else
    //
    // Now that we have a baud rate, we can configure our UART.
    //
    am_multiboot_setup_serial(AM_BSP_UART_BOOTLOADER_INST, MULTIBOOT_UART_BAUDRATE);
#endif

    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_BOOTLOADER_UART_TX, AM_BSP_GPIO_CFG_BOOTLOADER_UART_TX);
    am_hal_gpio_pin_config(AM_BSP_GPIO_BOOTLOADER_UART_RX, AM_BSP_GPIO_CFG_BOOTLOADER_UART_RX);
    //
    // Enable interrupts so we can receive messages from the boot host.
    //
    am_hal_interrupt_master_enable();
#endif

    //
    // Loop forever - should never reach here
    //
    while (1)
    {
    }
}
