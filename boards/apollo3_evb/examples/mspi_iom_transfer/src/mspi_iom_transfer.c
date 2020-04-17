//*****************************************************************************
//
//! @file mspi_iom_xfer.c
//!
//! @brief Example demonstrating the hardware assisted MSPI to IOM transfer
//!
//! Purpose: This example demonstrates transferring a large buffer from a flash device
//! connected on MSPI, to a FRAM device connected to IOM, using hardware
//! handshaking in Apollo3 - with minimal CPU involvement.
//!
//! At initialization, both the FRAM and Flash are initialized and a set pattern
//! data is written to the flash for transfer to FRAM.
//!
//! The FRAM is connected to IOM using SPI interface, and hence the transactions
//! are limited in size to 4095 Bytes.
//! The program here creates a command queue for both the MSPI and IOM, to
//! create a sequence of transactions - each reading a segment of the source
//! buffer to a temp buffer in internal SRAM, and then writing the same to the
//! FRAM using the IOM. It uses hardware handshaking so that the IOM transaction
//! is started only once the segement is read out completely from MSPI Flash.
//!
//! To best utilize the buses, a ping-pong model is used using two temporary
//! buffers in SRAM. This allows the interfaces to not idle while waiting for
//! other to finish - essentially achieving close to the bandwidth achieved by
//! the slower of the two.
//!
//! Printing takes place over the ITM at 1M Baud.
//!
//! Additional Information:
//! Configurable parameters at compile time:
//! IOM to use (FRAM_IOM_MODULE)
//! FRAM device to use (define one of FRAM_DEVICE_* to 1)
//! MSPI Flash to use - uses compile time definitions from am_device_mspi_flash.h
//! BLOCK_SIZE - total size of transaction
//! SPI_TXN_SIZE - size of temporary ping-pong buffer
//! CPU_SLEEP_GPIO - tracks CPU sleeping on analyzer
//! VERIFY_DATA - Enables reading back of the data from IOM to check & verify the accuracy
//! This can only be enabled if SEQLOOP is not being used
//!
//! Operating modes:
//! SEQLOOP not defined - The CQ is programmed each iteration
//! SEQLOOP - Advanced mode, to create sequence once, which repeats when triggered by callback at the end of each iteration
//! SEQLOOP - RUN_AUTONOMOUS - Sequence created once, and it repeats indefintely till paused (as a result of timer)
//! CQ_RAW - Uses Preconstructed CQ for IOM and MSPI - to save on the time to program the same at run time
//!
//! Best way to see the example in action is to connect logic analyzer and monitor the signals
//! Apart from the IO signals below, one can also monitor CPU_SLEEP_GPIO to monitor CPU in deep sleep
//! @verbatim
//! Pin connections:
//! IOM:
//! Particular IOM to use for this example is controlled by macro FRAM_IOM_MODULE
//! This example use apollo3_eb board connected to a fireball
//! Fireball is populated with MB85RS1MT SPI FRAM devices on each of the IOM's
//! with respective pin definitions in the BSP
//! Default pin settings for this example using IOM1 are:
//! #define AM_BSP_GPIO_IOM1_CS             14
//! #define AM_BSP_IOM1_CS_CHNL             2
//! #define AM_BSP_GPIO_IOM1_MISO           9
//! #define AM_BSP_GPIO_IOM1_MOSI           10
//! #define AM_BSP_GPIO_IOM1_SCK            8
//!
//! MSPI:
//! The MSPI flash device uses is controlled by macro FRAM_DEVICE_* (set one of them to 1)
//! This example uses apollo3_eb board connected to a fireball
//! Fireball is populated with CYPRESS_S25FS064S flash on CE0
//! #define AM_BSP_GPIO_MSPI_CE0            19
//! #define AM_BSP_MSPI_CE0_CHNL            0
//! #define AM_BSP_GPIO_MSPI_D0             22
//! #define AM_BSP_GPIO_MSPI_D1             26
//! #define AM_BSP_GPIO_MSPI_D2             4
//! #define AM_BSP_GPIO_MSPI_D3             23
//! #define AM_BSP_GPIO_MSPI_SCK            24
//!
//! And if the fireball device card is used, this example can work on:
//! Apollo3_eb + Fireball
//! Apollo3_eb + Fireball2
//! Recommend to use 1.8V power supply voltage.
//! Define FIREBALL_CARD or FIREBALL2_CARD in the config-template.ini file to select.
//! Define CYPRESS_S25FS064S or ADESTO_ATXP032 for Fireball
//! Define ADESTO_ATXP032 for Fireball2
//!
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
#if defined(ADESTO_ATXP032)
#include "am_devices_mspi_atxp032.h"
#define am_devices_mspi_flash_config_t am_devices_mspi_atxp032_config_t
#define AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE    AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE
#elif defined(CYPRESS_S25FS064S)
#include "am_devices_mspi_s25fs064s.h"
#define am_devices_mspi_flash_config_t am_devices_mspi_s25fs064s_config_t
#define AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS AM_DEVICES_MSPI_S25FS064S_STATUS_SUCCESS
#define AM_DEVICES_MSPI_FLASH_SECTOR_SIZE    AM_DEVICES_MSPI_S25FS064S_SECTOR_SIZE
#else
#error "Unknown FLASH Device"
#endif
#include "am_util.h"

#if FIREBALL_CARD || FIREBALL2_CARD
//
// The Fireball device card multiplexes various devices including each of an SPI
// and I2C FRAM. The Fireball device driver controls access to these devices.
// If the Fireball card is not used, FRAM devices can be connected directly
// to appropriate GPIO pins.
//
#include "am_devices_fireball.h"
#endif // FIREBALL_CARD

//*****************************************************************************
// Customize the following for the test
//*****************************************************************************
// Control the example execution
#define FRAM_IOM_MODULE         0
#define IOM_FREQ                AM_HAL_IOM_24MHZ
#define MSPI_FREQ               AM_HAL_MSPI_CLK_24MHZ

// Valid only if running in non-autonomous mode
// For autonomous mode - this indirectly controls the timer which triggers the stopping
#define NUM_ITERATIONS          32
// #define MSPI_FLASH_SERIAL // Configures flash in serial mode (default is quad)
// Control the size of the block of data to be transferred from Flash to FRAM
//#define BLOCK_SIZE              128*1024 // FRAM device is limited to 128K in size
#define BLOCK_SIZE              57600 // FRAM device is limited to 128K in size
// Size of temporary buffer being used
#define TEMP_BUFFER_SIZE        ((AM_HAL_IOM_MAX_TXNSIZE_SPI + 256) & 0xFFFFFF00)
// Size of SPI Transaction
#define SPI_TXN_SIZE            3840 // ((AM_HAL_IOM_MAX_TXNSIZE_SPI) & 0xFFFFFF00)
//#define SPI_TXN_SIZE            ((AM_HAL_IOM_MAX_TXNSIZE_SPI) & 0xFFFFFF00)
// Total number of SPI_TXN_SIZE fragments needs to transfer one full frame buffer
#define NUM_FRAGMENTS           ((BLOCK_SIZE + SPI_TXN_SIZE - 1) / SPI_TXN_SIZE)

#define DEFAULT_TIMEOUT         10000
//#define ENABLE_LOGGING

#define MSPI_TEST_MODULE        0

#define CPU_SLEEP_GPIO          29       // GPIO used to track CPU in sleep on logic analyzer
#define TEST_GPIO               7
#define TEST_GPIO1              28

// Whether to use CQ Loop feature
//#define SEQLOOP
#ifdef SEQLOOP
// Pause flag used for triggering MSPI sequence
// The sequence finishes one iteration and then waits for this flag to repeat
#define PAUSEFLAG                 0x4
// Keep running without CPU intervention in infinite loop
// If not defined - after each loop, CPU wakes up to trigger next sequence, and we stop after number of iterations
//#define RUN_AUTONOMOUS
#else
// Enables Verification of the FRAM data after it has been written to
// Can not be enabled if running in SEQLOOP mode
//#define VERIFY_DATA
#endif
// CQ Raw feature - enables preconstructed CQ using the raw interface
//#define CQ_RAW

// Enable reducing buffer size by this amount
#define VARIABLE_SIZE_CHANGE      0

// Select the FRAM Device
#ifdef FIREBALL_CARD
#define FRAM_DEVICE_MB85RS1MT     1     // SPI Fram (Fireball)
#elif FIREBALL2_CARD
#define FRAM_DEVICE_MB85RQ4ML     1     // SPI Fram (Fireball2)
#endif

//#define FRAM_DEVICE_MB85RS64V     1     // SPI Fram
//*****************************************************************************
//*****************************************************************************

#if (FRAM_DEVICE_MB85RS1MT == 1)
#include "am_devices_mb85rs1mt.h"
#define FRAM_DEVICE_ID          AM_DEVICES_MB85RS1MT_ID
#define am_iom_test_devices_t   am_devices_mb85rs1mt_config_t
#elif (FRAM_DEVICE_MB85RQ4ML == 1)
#include "am_devices_mb85rq4ml.h"
#define FRAM_DEVICE_ID          AM_DEVICES_MB85RQ4ML_ID
#define am_iom_test_devices_t   am_devices_mb85rq4ml_config_t
#elif (FRAM_DEVICE_MB85RS64V == 1)
#include "am_devices_mb85rs64v.h"
#define FRAM_DEVICE_ID          AM_DEVICES_MB85RS64V_ID
#define am_iom_test_devices_t   am_devices_mb85rs64v_config_t
#else
#error "Unknown FRAM Device"
#endif

// Helper Macros to map the ISR based on the IOM being used
#define FRAM_IOM_IRQn           ((IRQn_Type)(IOMSTR0_IRQn + FRAM_IOM_MODULE))

//
// Take over the interrupt handler for whichever IOM we're using.
//
#define fram_iom_isr                                                          \
    am_iom_isr1(FRAM_IOM_MODULE)
#define am_iom_isr1(n)                                                        \
    am_iom_isr(n)
#define am_iom_isr(n)                                                         \
    am_iomaster ## n ## _isr

// Friendlier names for the bit masks
#define AM_REG_IOM_CQFLAGS_CQFLAGS_MSPI1START   (_VAL2FLD(IOM0_CQPAUSEEN_CQPEN, IOM0_CQPAUSEEN_CQPEN_SWFLAGEN1))
#define AM_REG_IOM_CQFLAGS_CQFLAGS_MSPI0START   (_VAL2FLD(IOM0_CQPAUSEEN_CQPEN, IOM0_CQPAUSEEN_CQPEN_SWFLAGEN0))

#define IOM_SIGNAL_MSPI_BUFFER0       (AM_REG_IOM_CQFLAGS_CQFLAGS_MSPI0START << 8)
#define IOM_SIGNAL_MSPI_BUFFER1       (AM_REG_IOM_CQFLAGS_CQFLAGS_MSPI1START << 8)
#define MSPI_SIGNAL_IOM_BUFFER0       (MSPI_CQFLAGS_CQFLAGS_SWFLAG0 << 8)
#define MSPI_SIGNAL_IOM_BUFFER1       (MSPI_CQFLAGS_CQFLAGS_SWFLAG1 << 8)

#define IOM_WAIT_FOR_MSPI_BUFFER0     (_VAL2FLD(IOM0_CQPAUSEEN_CQPEN, IOM0_CQPAUSEEN_CQPEN_MSPI0XNOREN))
#define IOM_WAIT_FOR_MSPI_BUFFER1     (_VAL2FLD(IOM0_CQPAUSEEN_CQPEN, IOM0_CQPAUSEEN_CQPEN_MSPI1XNOREN))
#define MSPI_WAIT_FOR_IOM_BUFFER0     (_VAL2FLD(MSPI_CQFLAGS_CQFLAGS, MSPI_CQFLAGS_CQFLAGS_IOM0READY))
#define MSPI_WAIT_FOR_IOM_BUFFER1     (_VAL2FLD(MSPI_CQFLAGS_CQFLAGS, MSPI_CQFLAGS_CQFLAGS_IOM1READY))


#ifdef ENABLE_LOGGING
#define DEBUG_PRINT am_util_stdio_printf
#else
#define DEBUG_PRINT(...)
#endif

#define DEBUG_GPIO_HIGH(gpio)   am_hal_gpio_fastgpio_set(gpio)
#define DEBUG_GPIO_LOW(gpio)    am_hal_gpio_fastgpio_clr(gpio)
//
// Typedef - to encapsulate device driver functions
//
typedef struct
{
    uint8_t  devName[20];
    uint32_t (*fram_init)(uint32_t ui32Module, am_iom_test_devices_t *pDevConfig, void **ppHandle, void **ppIomHandle);
    uint32_t (*fram_term)(void *pHandle);

    uint32_t (*fram_read_id)(void *pHandle, uint32_t *pDeviceID);

    uint32_t (*fram_blocking_write)(void *pHandle, uint8_t *ui8TxBuffer,
                             uint32_t ui32WriteAddress,
                             uint32_t ui32NumBytes);

    uint32_t (*fram_nonblocking_write)(void *pHandle, uint8_t *ui8TxBuffer,
                                uint32_t ui32WriteAddress,
                                uint32_t ui32NumBytes,
                                am_hal_iom_callback_t pfnCallback,
                                void *pCallbackCtxt);

    uint32_t (*fram_nonblocking_write_adv)(void *pHandle, uint8_t *ui8TxBuffer,
                                uint32_t ui32WriteAddress,
                                uint32_t ui32NumBytes,
                                uint32_t ui32PauseCondition,
                                uint32_t ui32StatusSetClr,
                                am_hal_iom_callback_t pfnCallback,
                                void *pCallbackCtxt);

    uint32_t (*fram_blocking_read)(void *pHandle, uint8_t *pui8RxBuffer,
                            uint32_t ui32ReadAddress,
                            uint32_t ui32NumBytes);

    uint32_t (*fram_nonblocking_read)(void *pHandle, uint8_t *pui8RxBuffer,
                                                      uint32_t ui32ReadAddress,
                                                      uint32_t ui32NumBytes,
                                                      am_hal_iom_callback_t pfnCallback,
                                                      void *pCallbackCtxt);
    uint32_t (*fram_command_send)(void *pHandle, uint32_t ui32Cmd);
    am_devices_fireball_control_e fram_fireball_control;
} fram_device_func_t;

//
// Typedef - to encapsulate device driver functions
//
typedef struct
{
    uint8_t  devName[20];
    uint32_t (*flash_init)(uint32_t ui32Module, am_devices_mspi_flash_config_t *pDevConfig, void **ppHandle, void **ppMspiHandle);
    uint32_t (*flash_term)(void *pHandle);

    uint32_t (*flash_read_id)(void *pHandle);

    uint32_t (*flash_write)(void *pHandle, uint8_t *pui8TxBuffer,
                            uint32_t ui32WriteAddress,
                            uint32_t ui32NumBytes,
                            bool bWaitForCompletion);

    uint32_t (*flash_read)(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);

    uint32_t (*flash_read_adv)(void *pHandle, uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt);

    uint32_t (*flash_mass_erase)(void *pHandle);
    uint32_t (*flash_sector_erase)(void *pHandle, uint32_t ui32SectorAddress);

    uint32_t (*flash_enable_xip)(void *pHandle);
    uint32_t (*flash_disable_xip)(void *pHandle);
    uint32_t (*flash_enable_scrambling)(void *pHandle);
    uint32_t (*flash_disable_scrambling)(void *pHandle);

    am_devices_fireball_control_e flash_fireball_control;
} flash_device_func_t;

// Globals
#ifndef CQ_RAW
// Buffer for non-blocking transactions for IOM - Needs to be big enough to accomodate
// all the transactions
// Each IOM transafer on FRAM takes three transactions
uint32_t        g_IomQBuffer[(AM_HAL_IOM_CQ_ENTRY_SIZE / 4) * (3 * (NUM_FRAGMENTS + 1))];
// Buffer for non-blocking transactions for MSPI - Needs to be big enough to accomodate
// all the transactions
uint32_t        g_MspiQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (NUM_FRAGMENTS + 1)];
#else
// Buffer for non-blocking transactions for IOM - can be much smaller as the CQ is preconstructed in a separare memory
// all the transactions
//uint32_t        g_IomQBuffer[(AM_HAL_IOM_CQ_ENTRY_SIZE / 4) * (4 + 1)];
// Buffer for non-blocking transactions for MSPI - can be much smaller as the CQ is preconstructed in a separare memory
// all the transactions
uint32_t        g_MspiQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * (4 + 1)];
#endif

// Temp Buffer in SRAM to read PSRAM data to, and write DISPLAY data from
uint32_t        g_TempBuf[2][TEMP_BUFFER_SIZE / 4];

void            *g_FlashHdl;
void            *g_MSPIHdl;
void            *g_IomDevHdl;
void            *g_IOMHandle;
volatile bool   g_bDone = false;

uint32_t      numIter = 0;

uint32_t g_FramChipSelect[AM_REG_IOM_NUM_MODULES] =
{
    AM_BSP_IOM0_CS_CHNL,
    AM_BSP_IOM1_CS_CHNL,
#ifndef APOLLO3_FPGA
    AM_BSP_IOM2_CS_CHNL,
    AM_BSP_IOM3_CS_CHNL,
    AM_BSP_IOM4_CS_CHNL,
    AM_BSP_IOM5_CS_CHNL,
#endif
};

am_devices_mspi_flash_config_t MSPI_Flash_Config = 
{
#ifdef MSPI_FLASH_SERIAL
    .eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0,
#else
    .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0,
#endif
    .eClockFreq = MSPI_FREQ,
    .eMixedMode = AM_HAL_MSPI_XIPMIXED_NORMAL,
    .pNBTxnBuf = g_MspiQBuffer,
    .ui32NBTxnBufLength = (sizeof(g_MspiQBuffer) / sizeof(uint32_t)),
    .ui32ScramblingStartAddr = 0,
    .ui32ScramblingEndAddr = 0,
};

fram_device_func_t fram_func =
{
#if (FRAM_DEVICE_MB85RS1MT == 1)
    // Fireball installed SPI FRAM device
    .devName = "SPI FRAM MB85RS1MT",
    .fram_init = am_devices_mb85rs1mt_init,
    .fram_term = am_devices_mb85rs1mt_term,
    .fram_read_id = am_devices_mb85rs1mt_read_id,
    .fram_blocking_write = am_devices_mb85rs1mt_blocking_write,
    .fram_nonblocking_write = am_devices_mb85rs1mt_nonblocking_write,
    .fram_nonblocking_write_adv = am_devices_mb85rs1mt_nonblocking_write_adv,
    .fram_blocking_read = am_devices_mb85rs1mt_blocking_read,
    .fram_nonblocking_read = am_devices_mb85rs1mt_nonblocking_read,
    .fram_command_send = am_devices_mb85rs1mt_command_send,
#if FIREBALL_CARD
    .fram_fireball_control = AM_DEVICES_FIREBALL_STATE_SPI_FRAM,
#else
    .fram_fireball_control = 0,
#endif
#elif (FRAM_DEVICE_MB85RQ4ML == 1)
    // Fireball installed SPI FRAM device
    .devName = "SPI FRAM MB85RQ4ML",
    .fram_init = am_devices_mb85rq4ml_init,
    .fram_term = am_devices_mb85rq4ml_term,
    .fram_read_id = am_devices_mb85rq4ml_read_id,
    .fram_blocking_write = am_devices_mb85rq4ml_blocking_write,
    .fram_nonblocking_write = am_devices_mb85rq4ml_nonblocking_write,
    .fram_nonblocking_write_adv = am_devices_mb85rq4ml_nonblocking_write_adv,
    .fram_blocking_read = am_devices_mb85rq4ml_blocking_read,
    .fram_nonblocking_read = am_devices_mb85rq4ml_nonblocking_read,
    .fram_command_send = am_devices_mb85rq4ml_command_send,
#if FIREBALL2_CARD
    .fram_fireball_control = AM_DEVICES_FIREBALL2_STATE_SPI_FRAM_PSRAM_1P8,
#else
    .fram_fireball_control = 0,
#endif
#elif (FRAM_DEVICE_MB85RS64V == 1)
    .devName = "SPI FRAM MB85RS64V",
    .fram_init = am_devices_mb85rs64v_init,
    .fram_term = am_devices_mb85rs64v_term,
    .fram_read_id = am_devices_mb85rs64v_read_id,
    .fram_blocking_write = am_devices_mb85rs64v_blocking_write,
    .fram_nonblocking_write = am_devices_mb85rs64v_nonblocking_write,
    .fram_nonblocking_write_adv = am_devices_mb85rs64v_nonblocking_write_adv,
    .fram_blocking_read = am_devices_mb85rs64v_blocking_read,
    .fram_nonblocking_read = am_devices_mb85rs64v_nonblocking_read,
    .fram_command_send = am_devices_mb85rs64v_command_send,
    .fram_fireball_control = 0,
#else
#error "Unknown FRAM Device"
#endif
};

flash_device_func_t flash_func =
{
#if defined(ADESTO_ATXP032)
    // Fireball installed MSPI FLASH device
    .devName = "MSPI FLASH ATXP032",
    .flash_init = am_devices_mspi_atxp032_init,
    .flash_term = am_devices_mspi_atxp032_deinit,
    .flash_read_id = am_devices_mspi_atxp032_id,
    .flash_write = am_devices_mspi_atxp032_write,
    .flash_read = am_devices_mspi_atxp032_read,
    .flash_read_adv = am_devices_mspi_atxp032_read_adv,
    .flash_mass_erase = am_devices_mspi_atxp032_mass_erase,
    .flash_sector_erase = am_devices_mspi_atxp032_sector_erase,
    .flash_enable_xip = am_devices_mspi_atxp032_enable_xip,
    .flash_disable_xip = am_devices_mspi_atxp032_disable_xip,
    .flash_enable_scrambling = am_devices_mspi_atxp032_enable_scrambling,
    .flash_disable_scrambling = am_devices_mspi_atxp032_disable_scrambling,
#if FIREBALL_CARD
    .flash_fireball_control = AM_DEVICES_FIREBALL_STATE_OCTAL_FLASH_CE0,
#else
    .flash_fireball_control = 0,
#endif
#elif defined(CYPRESS_S25FS064S)
    // Fireball installed MSPI FLASH device
    .devName = "MSPI FLASH S25FS064S",
    .flash_init = am_devices_mspi_s25fs064s_init,
    .flash_term = am_devices_mspi_s25fs064s_deinit,
    .flash_read_id = am_devices_mspi_s25fs064s_id,
    .flash_write = am_devices_mspi_s25fs064s_write,
    .flash_read = am_devices_mspi_s25fs064s_read,
    .flash_read_adv = am_devices_mspi_s25fs064s_read_adv,
    .flash_mass_erase = am_devices_mspi_s25fs064s_mass_erase,
    .flash_sector_erase = am_devices_mspi_s25fs064s_sector_erase,
    .flash_enable_xip = am_devices_mspi_s25fs064s_enable_xip,
    .flash_disable_xip = am_devices_mspi_s25fs064s_disable_xip,
    .flash_enable_scrambling = am_devices_mspi_s25fs064s_enable_scrambling,
    .flash_disable_scrambling = am_devices_mspi_s25fs064s_disable_scrambling,
#if FIREBALL_CARD
    .flash_fireball_control = AM_DEVICES_FIREBALL_STATE_TWIN_QUAD_CE0_CE1,
#else
    .flash_fireball_control = 0,
#endif
#else
#error "Unknown FLASH Device"
#endif
};

//! MSPI interrupts.
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
#if defined(AM_PART_APOLLO3P)
    MSPI1_IRQn,
    MSPI2_IRQn,
#endif
};

//
// Take over the interrupt handler for whichever MSPI we're using.
//
#define flash_mspi_isr                                                          \
    am_mspi_isr1(MSPI_TEST_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void flash_mspi_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIHdl, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIHdl, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIHdl, ui32Status);
}


#if defined(SEQLOOP) && defined(RUN_AUTONOMOUS)
//**************************************
// Timer configuration.
//**************************************
am_hal_ctimer_config_t g_sTimer0 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_ONCE    |
     AM_HAL_CTIMER_INT_ENABLE   |
#if USE_XTAL
     AM_HAL_CTIMER_XT_256HZ),
#else
     AM_HAL_CTIMER_LFRC_32HZ),
#endif

    // No configuration for Timer0B.
    0,
};

//*****************************************************************************
//
// Function to initialize Timer A0 to interrupt every 1/4 second.
//
//*****************************************************************************
void
timerA0_init(void)
{
    uint32_t ui32Period;

    //
    // Enable the LFRC.
    //
#if USE_XTAL
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
#else
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
#endif

    //
    // Set up timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer0);

    //
    // Set up timerA0 to 32Hz from LFRC divided to 1 second period.
    //
    ui32Period = 32 * NUM_ITERATIONS;
#if USE_XTAL
    ui32Period *= 8;
#endif
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
                             (ui32Period >> 1));

    //
    // Clear the timer Interrupt
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
    //
    // Enable the timer Interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_EnableIRQ(CTIMER_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Start timer A0
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear TimerA0 Interrupt (write to clear).
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
    // Cause the loop to terminate
    DEBUG_PRINT("Timer expired. Terminating the loop after %d sec\n", NUM_ITERATIONS);
    uint32_t flag = PAUSEFLAG << 16;
    am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_FLAG_SETCLR, &flag);
    numIter = NUM_ITERATIONS;
    g_bDone = 1;
}
#endif

//*****************************************************************************
//
// IOM ISRs.
//
//*****************************************************************************
//
//! Take over default ISR. (Queue mode service)
//
void fram_iom_isr(void)
{
    uint32_t ui32Status;

    if (!am_hal_iom_interrupt_status_get(g_IOMHandle, true, &ui32Status))
    {
        if ( ui32Status )
        {
            am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
            am_hal_iom_interrupt_service(g_IOMHandle, ui32Status);
        }
    }
}

int
fireball_init (void)
{
    uint32_t ui32Ret, ui32ID;

    //
    // Get Fireball ID and Rev info.
    //
    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_ID_GET, &ui32ID);
    if ( ui32Ret != 0 )
    {
        DEBUG_PRINT("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_ID_GET, ui32Ret);
        return -1;
    }
    else if ( ui32ID == FIREBALL_ID )
    {
        DEBUG_PRINT("Fireball found, ID is 0x%X.\n", ui32ID);
    }
    else if ( ui32ID == FIREBALL2_ID )
    {
        DEBUG_PRINT("Fireball 2 found, ID is 0x%X.\n", ui32ID);
    }
    else
    {
        DEBUG_PRINT("Unknown device returned ID as 0x%X.\n", ui32ID);
    }

    ui32Ret = am_devices_fireball_control(AM_DEVICES_FIREBALL_STATE_VER_GET, &ui32ID);
    if ( ui32Ret != 0 )
    {
        DEBUG_PRINT("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_VER_GET, ui32Ret);
        return -1;
    }
    else
    {
        DEBUG_PRINT("Fireball Version is 0x%X.\n", ui32ID);
    }
    return 0;
}

int
fram_init(void)
{
    uint32_t ui32Status;
    uint32_t ui32DeviceId;

    // Set up IOM
    // Initialize the Device

#if FIREBALL_CARD || FIREBALL2_CARD
    uint32_t ui32Ret;

    if ( fram_func.fram_fireball_control != 0 )
    {
        ui32Ret = am_devices_fireball_control(fram_func.fram_fireball_control, 0);
        if ( ui32Ret != 0 )
        {
            DEBUG_PRINT("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                                 fram_func.fram_fireball_control, ui32Ret);
            return -1;
        }
    }
#endif // FIREBALL_CARD

    am_iom_test_devices_t stFramConfig;
    stFramConfig.ui32ClockFreq = IOM_FREQ;
    stFramConfig.pNBTxnBuf = g_IomQBuffer;
    stFramConfig.ui32NBTxnBufLength = sizeof(g_IomQBuffer) / 4;

    ui32Status = fram_func.fram_init(FRAM_IOM_MODULE, &stFramConfig, &g_IomDevHdl, &g_IOMHandle);
    if (0 == ui32Status)
    {
        ui32Status = fram_func.fram_read_id(g_IomDevHdl, &ui32DeviceId);

        if ((ui32Status  != 0) || (ui32DeviceId != FRAM_DEVICE_ID))
        {
            return -1;
        }
        DEBUG_PRINT("%s Found\n", fram_func.devName);
    }
    else
    {
        return -1;
    }
    //
    // Enable the IOM interrupt in the NVIC.
    //
    NVIC_EnableIRQ(FRAM_IOM_IRQn);
    return 0;
}

int
fram_deinit(void)
{
    uint32_t ui32Status;

    //
    // Disable the IOM interrupt in the NVIC.
    //
    NVIC_DisableIRQ(FRAM_IOM_IRQn);

    // Set up IOM
    // Initialize the Device

    ui32Status = fram_func.fram_term(g_IomDevHdl);
    if (0 != ui32Status)
    {
        DEBUG_PRINT("Failed to terminate FRAM device\n");
        return -1;
    }
    return 0;
}

int
mspi_flash_init(am_devices_mspi_flash_config_t mspiFlashConfig)
{

    uint32_t      ui32Status;
#if FIREBALL_CARD || FIREBALL2_CARD // Set the fireball card for MSPI
    //
    // Set the MUX for the Flash Device
    //
    uint32_t ui32Ret;

    ui32Ret = am_devices_fireball_control(flash_func.flash_fireball_control, 0);
    if ( ui32Ret != 0 )
    {
        DEBUG_PRINT("FAIL: am_devices_fireball_control(%d) returned 0x%X.\n",
                             AM_DEVICES_FIREBALL_STATE_TWIN_QUAD_CE0_CE1, ui32Ret);
        return -1;
    }
#endif // FIREBALL_CARD

    //
    // Configure the MSPI and Flash Device.
    //
    ui32Status = flash_func.flash_init(MSPI_TEST_MODULE, &mspiFlashConfig, &g_FlashHdl, &g_MSPIHdl);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to configure the MSPI and Flash Device correctly!\n");
        return -1;
    }
    NVIC_EnableIRQ(mspi_interrupts[MSPI_TEST_MODULE]);

#if !defined (ADESTO_ATXP032) // Not all flash devices support Read ID in quad mode
    //
    // Read the MSPI Device ID.
    //
    ui32Status = flash_func.flash_read_id(g_FlashHdl);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to read Flash Device ID!\n");
        return -1;
    }
#endif
    //
    // Make sure we aren't in XIP mode.
    //
    ui32Status = flash_func.flash_disable_xip(g_FlashHdl);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to disable XIP mode in the MSPI!\n");
        return -1;
    }
    return 0;
}

int
init_mspi_flash_data(void)
{
    uint32_t      ui32Status;

    // Initialize the flash (128K == SPI FRAM size) with the pattern data
#if !defined (ADESTO_ATXP032)
    // Mass Erase
    am_util_stdio_printf("Initiating mass erase Flash Device! - This can take a long time\n");
    ui32Status = flash_func.flash_mass_erase(g_FlashHdl);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to mass erase Flash Device!\n");
        return -1;
    }
    am_util_stdio_printf("mass erase Flash Device Done!\n");
#else
    // Erase the sectors we need to program
    am_util_stdio_printf("Initiating erase of required sectors of Flash Device!\n");
    for (uint32_t address = 0; address < BLOCK_SIZE; address += AM_DEVICES_MSPI_FLASH_SECTOR_SIZE)
    {
        uint32_t sector = address / AM_DEVICES_MSPI_FLASH_SECTOR_SIZE;
        //
        // Erase the target sector.
        //
        am_util_stdio_printf("Erasing Sector %d\n", sector);
        ui32Status = flash_func.flash_sector_erase(g_FlashHdl, sector << 16);
        if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to erase Flash Device sector!\n");
            return -1;
        }
    }
    am_util_stdio_printf("Flash Device Erase done!\n");
#endif

    am_util_stdio_printf("Writing a known pattern to flash!\n");
    for (uint32_t address = 0; address < BLOCK_SIZE; address += TEMP_BUFFER_SIZE)
    {
        //
        // Generate data into the Sector Buffer
        //
        for (uint32_t i = 0; i < TEMP_BUFFER_SIZE / 4; i++)
        {
            g_TempBuf[0][i] = address + i*4;
        }

        //
        // Write the TX buffer into the target sector.
        //
        ui32Status = flash_func.flash_write(g_FlashHdl, (uint8_t *)g_TempBuf[0], address, TEMP_BUFFER_SIZE, true);
        if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to write buffer to Flash Device!\n");
            return -1;
        }
        //
        // Read the data back into the RX buffer.
        //
        ui32Status = flash_func.flash_read(g_FlashHdl, (uint8_t *)g_TempBuf[1], address, TEMP_BUFFER_SIZE, true);
        if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
        }

        //
        // Compare the buffers
        //
        for (uint32_t i = 0; i < TEMP_BUFFER_SIZE / 4; i++)
        {
            if (g_TempBuf[1][i] != g_TempBuf[0][i])
            {
                am_util_stdio_printf("TX and RX buffers failed to compare!\n");
                return -1;
            }
        }
    }
    return 0;
}

void
flash_read_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        DEBUG_PRINT("\nIter# %d:Flash Read Failed 0x%x\n", numIter, transactionStatus);
    }
    else
    {
        DEBUG_PRINT("\nIter# %d:Flash Read Done 0x%x\n", numIter, transactionStatus);
    }
}

void
fram_write_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    if (transactionStatus != AM_HAL_STATUS_SUCCESS)
    {
        DEBUG_PRINT("\nIter# %d:FRAM Write Failed 0x%x\n", numIter, transactionStatus);
    }
    else
    {
        DEBUG_PRINT("\nIter# %d:FRAM Write Done 0x%x\n", numIter, transactionStatus);
        g_bDone = true;
    }
}

#ifdef CQ_RAW
// 2 blocks are includes in head and tail
#define MAX_INT_BLOCKS    (NUM_FRAGMENTS - 2)
// Jump - by reprogramming the CQADDR
typedef struct
{
    uint32_t    ui32CQAddrAddr;
    uint32_t    ui32CQAddrVal;
} am_hal_cq_jmp_t;

// IOM
//
// Command Queue entry structure.
//
typedef struct
{
    uint32_t    ui32PAUSENAddr;
    uint32_t    ui32PAUSEENVal;
    uint32_t    ui32PAUSEN2Addr;
    uint32_t    ui32PAUSEEN2Val;
    // Following 4 fields are only needed for first block
    uint32_t    ui32OFFSETHIAddr;
    uint32_t    ui32OFFSETHIVal;
    uint32_t    ui32DEVCFGAddr;
    uint32_t    ui32DEVCFGVal;

    uint32_t    ui32DMACFGdis1Addr;
    uint32_t    ui32DMACFGdis1Val;
    uint32_t    ui32DMATOTCOUNTAddr;
    uint32_t    ui32DMATOTCOUNTVal;
    uint32_t    ui32DMATARGADDRAddr;
    uint32_t    ui32DMATARGADDRVal;
    uint32_t    ui32DMACFGAddr;
    uint32_t    ui32DMACFGVal;
    // CMDRPT register has been repurposed for DCX
    uint32_t    ui32DCXAddr;
    uint32_t    ui32DCXVal;
    uint32_t    ui32CMDAddr;
    uint32_t    ui32CMDVal;

    uint32_t    ui32SETCLRAddr;
    uint32_t    ui32SETCLRVal;
} am_hal_iom_txn_head_t;

typedef struct
{
    uint32_t    ui32PAUSENAddr;
    uint32_t    ui32PAUSEENVal;
    uint32_t    ui32PAUSEN2Addr;
    uint32_t    ui32PAUSEEN2Val;

    uint32_t    ui32DMACFGdis1Addr;
    uint32_t    ui32DMACFGdis1Val;
    uint32_t    ui32DMATOTCOUNTAddr;
    uint32_t    ui32DMATOTCOUNTVal;
    uint32_t    ui32DMATARGADDRAddr;
    uint32_t    ui32DMATARGADDRVal;
    uint32_t    ui32DMACFGAddr;
    uint32_t    ui32DMACFGVal;
    // CMDRPT register has been repurposed for DCX
    uint32_t    ui32DCXAddr;
    uint32_t    ui32DCXVal;
    uint32_t    ui32CMDAddr;
    uint32_t    ui32CMDVal;

    uint32_t    ui32SETCLRAddr;
    uint32_t    ui32SETCLRVal;
} am_hal_iom_txn_seg_t;

typedef struct
{
    // Each block will be max size transfer with CONT set
    am_hal_iom_txn_seg_t block[MAX_INT_BLOCKS]; // Fixed
    am_hal_cq_jmp_t  jmp; // Jump always to am_hal_iom_long_txn_t->tail
} am_hal_iom_txn_seq_t;

typedef struct
{
    // Head
    // Max size transfer with CONT Set
    am_hal_iom_txn_head_t    head; // Programmable per transaction: ui32OFFSETHIVal, ui32HdrDMATARGADDRVal, ui32HdrCMDVal
    // Jmp
    am_hal_cq_jmp_t          jmp; // Programmable address to jump inside am_hal_iom_txn_seq_t
    // Tail
    am_hal_iom_txn_seg_t     tail; // Programmable tail length, CMD Value
    am_hal_cq_jmp_t          jmpOut; // Programmable address to jump back to the original CQ
} am_hal_iom_long_txn_t;

// MSPI
//
// Command Queue entry structure.
//
typedef struct
{
    uint32_t                    ui32PAUSENAddr;
    uint32_t                    ui32PAUSEENVal;
    uint32_t                    ui32PAUSEN2Addr;
    uint32_t                    ui32PAUSEEN2Val;
    uint32_t                    ui32DMATARGADDRAddr;
    uint32_t                    ui32DMATARGADDRVal;
    uint32_t                    ui32DMADEVADDRAddr;
    uint32_t                    ui32DMADEVADDRVal;
    uint32_t                    ui32DMATOTCOUNTAddr;
    uint32_t                    ui32DMATOTCOUNTVal;
    uint32_t                    ui32DMACFG1Addr;
    uint32_t                    ui32DMACFG1Val;
    // Need to disable the DMA to prepare for next reconfig
    // Need to have this following the DMAEN for CMDQ
    uint32_t                    ui32DMACFG2Addr;
    uint32_t                    ui32DMACFG2Val;
    uint32_t                    ui32SETCLRAddr;
    uint32_t                    ui32SETCLRVal;
} am_hal_mspi_txn_seg_t;

typedef struct
{
    // Each block will be max size transfer with CONT set
    am_hal_mspi_txn_seg_t block[MAX_INT_BLOCKS]; // Fixed
    am_hal_cq_jmp_t  jmp; // Jump always to am_hal_iom_long_txn_t->tail
} am_hal_mspi_txn_seq_t;


typedef struct
{
    // Head
    // Max size transfer with CONT Set
    am_hal_mspi_txn_seg_t    head; // Programmable per transaction: ui32OFFSETHIVal, ui32HdrDMATARGADDRVal, ui32HdrCMDVal
    // Jmp
    am_hal_cq_jmp_t      jmp; // Programmable address to jump inside am_hal_iom_txn_seq_t
    // Tail
    am_hal_mspi_txn_seg_t     tail; // Programmable tail length, CMD Value
    am_hal_cq_jmp_t      jmpOut; // Programmable address to jump back to the original CQ
} am_hal_mspi_long_txn_t;

am_hal_iom_long_txn_t  gIomLongTxn;
am_hal_iom_txn_seq_t   gIomSequence;
am_hal_mspi_long_txn_t gMspiLongTxn;
am_hal_mspi_txn_seq_t  gMspiSequence;

//*****************************************************************************
//
// Function to build the CMD value.
// Returns the CMD value, but does not set the CMD register.
//
// The OFFSETHI register must still be handled by the caller, e.g.
//      AM_REGn(IOM, ui32Module, OFFSETHI) = (uint16_t)(ui32Offset >> 8);
//
//*****************************************************************************
static uint32_t
build_iom_cmd(uint32_t ui32CS,     uint32_t ui32Dir, uint32_t ui32Cont,
          uint32_t ui32Offset, uint32_t ui32OffsetCnt,
          uint32_t ui32nBytes)
{
    //
    // Initialize the CMD variable
    //
    uint32_t ui32Cmd = 0;

    //
    // If SPI, we'll need the chip select
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_CMDSEL, ui32CS);

    //
    // Build the CMD with number of bytes and direction.
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_TSIZE, ui32nBytes);

    if (ui32Dir == AM_HAL_IOM_RX)
    {
        ui32Cmd |= _VAL2FLD(IOM0_CMD_CMD, IOM0_CMD_CMD_READ);
    }
    else
    {
        ui32Cmd |= _VAL2FLD(IOM0_CMD_CMD, IOM0_CMD_CMD_WRITE);
    }

    ui32Cmd |= _VAL2FLD(IOM0_CMD_CONT, ui32Cont);

    //
    // Now add the OFFSETLO and OFFSETCNT information.
    //
    ui32Cmd |= _VAL2FLD(IOM0_CMD_OFFSETLO, (uint8_t)ui32Offset);
    ui32Cmd |= _VAL2FLD(IOM0_CMD_OFFSETCNT, ui32OffsetCnt);

    return ui32Cmd;
} // build_cmd()


// One time initialization
void iom_init_cq_long(uint32_t ui32Module)
{
    am_hal_iom_long_txn_t *pIomLong = &gIomLongTxn;
    // Initialize the head block
    // Initialize the tail block
    //
    // Command for OFFSETHI
    //
    pIomLong->head.ui32OFFSETHIAddr  = (uint32_t)&IOMn(ui32Module)->OFFSETHI;

    //
    // Command for I2C DEVADDR field in DEVCFG
    //
    pIomLong->head.ui32DEVCFGAddr    = (uint32_t)&IOMn(ui32Module)->DEVCFG;

    //
    // Command to disable DMA before writing TOTCOUNT.
    //
    pIomLong->tail.ui32DMACFGdis1Addr = pIomLong->head.ui32DMACFGdis1Addr   = (uint32_t)&IOMn(ui32Module)->DMACFG;
    pIomLong->tail.ui32DMACFGdis1Val = pIomLong->head.ui32DMACFGdis1Val    = 0x0;

    //
    // Command to set DMATOTALCOUNT
    //
    pIomLong->tail.ui32DMATOTCOUNTAddr = pIomLong->head.ui32DMATOTCOUNTAddr = (uint32_t)&IOMn(ui32Module)->DMATOTCOUNT;

    //
    // Command to set DMATARGADDR
    //
    pIomLong->head.ui32DMATARGADDRAddr = (uint32_t)&IOMn(ui32Module)->DMATARGADDR;
    pIomLong->tail.ui32DMATARGADDRAddr = (uint32_t)&IOMn(ui32Module)->DMATARGADDR;
    pIomLong->tail.ui32DMATARGADDRVal = (MAX_INT_BLOCKS % 2) ? (uint32_t)&g_TempBuf[0] : (uint32_t)&g_TempBuf[1];
    //
    // Command to set DMACFG to start the DMA operation
    //
    pIomLong->tail.ui32DMACFGAddr = pIomLong->head.ui32DMACFGAddr    = (uint32_t)&IOMn(ui32Module)->DMACFG;

    // CMDRPT register has been repurposed for DCX
    pIomLong->head.ui32DCXAddr = pIomLong->tail.ui32DCXAddr = (uint32_t)&IOMn(ui32Module)->DCX;
    pIomLong->head.ui32DCXVal = pIomLong->tail.ui32DCXVal = 0; // This will be replaced with DCX Alternate CS TODO

    pIomLong->tail.ui32CMDAddr = pIomLong->head.ui32CMDAddr = (uint32_t)&IOMn(ui32Module)->CMD;

    // Initialize the jumps
    gIomSequence.jmp.ui32CQAddrAddr = pIomLong->jmpOut.ui32CQAddrAddr = pIomLong->jmp.ui32CQAddrAddr = (uint32_t)&IOMn(ui32Module)->CQADDR;
    gIomSequence.jmp.ui32CQAddrVal = (uint32_t)&pIomLong->tail;
    // Initialize the sequence blocks
    for (uint32_t i = 0; i < MAX_INT_BLOCKS; i++)
    {
        gIomSequence.block[i].ui32PAUSENAddr = gIomSequence.block[i].ui32PAUSEN2Addr = (uint32_t)&IOMn(ui32Module)->CQPAUSEEN;
        gIomSequence.block[i].ui32SETCLRAddr = (uint32_t)&IOMn(ui32Module)->CQSETCLEAR;
        //
        // Command to disable DMA before writing TOTCOUNT.
        //
        gIomSequence.block[i].ui32DMACFGdis1Addr = (uint32_t)&IOMn(ui32Module)->DMACFG;
        gIomSequence.block[i].ui32DMACFGdis1Val = 0x0;

        //
        // Command to set DMATOTALCOUNT
        //
        gIomSequence.block[i].ui32DMATOTCOUNTAddr = (uint32_t)&IOMn(ui32Module)->DMATOTCOUNT;

        //
        // Command to set DMACFG to start the DMA operation
        //
        gIomSequence.block[i].ui32DMACFGAddr = (uint32_t)&IOMn(ui32Module)->DMACFG;

        gIomSequence.block[i].ui32DCXAddr = (uint32_t)&IOMn(ui32Module)->DCX;
        gIomSequence.block[i].ui32CMDAddr = (uint32_t)&IOMn(ui32Module)->CMD;
        // Pause Conditions
#if 0
        // Normal Memory
        // Set default values - these may be updated later
        gIomSequence.block[i].ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT;
        gIomSequence.block[i].ui32SETCLRVal = 0;
#else
        // This is the Pause Boundary for HiPrio transactions
        gIomSequence.block[i].ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT | ((i % 2) ? IOM_WAIT_FOR_MSPI_BUFFER0 : IOM_WAIT_FOR_MSPI_BUFFER1);
        gIomSequence.block[i].ui32PAUSEEN2Val = AM_HAL_IOM_PAUSE_DEFAULT;
        gIomSequence.block[i].ui32SETCLRVal = (i % 2) ? IOM_SIGNAL_MSPI_BUFFER0 : IOM_SIGNAL_MSPI_BUFFER1;
#endif
        gIomSequence.block[i].ui32DMATARGADDRAddr = (uint32_t)&IOMn(ui32Module)->DMATARGADDR;
        gIomSequence.block[i].ui32DMATARGADDRVal = (i % 2) ? (uint32_t)&g_TempBuf[0] : (uint32_t)&g_TempBuf[1];
    }
    // Initialize the Pause conditions
    pIomLong->head.ui32PAUSENAddr = pIomLong->tail.ui32PAUSENAddr = (uint32_t)&IOMn(ui32Module)->CQPAUSEEN;
    pIomLong->head.ui32PAUSEN2Addr = pIomLong->tail.ui32PAUSEN2Addr = (uint32_t)&IOMn(ui32Module)->CQPAUSEEN;
    pIomLong->head.ui32SETCLRAddr = pIomLong->tail.ui32SETCLRAddr = (uint32_t)&IOMn(ui32Module)->CQSETCLEAR;

    pIomLong->tail.ui32PAUSEEN2Val = AM_HAL_IOM_PAUSE_DEFAULT;
    pIomLong->head.ui32PAUSEEN2Val = AM_HAL_IOM_PAUSE_DEFAULT;
#if 0
    // Normal Memory
    // Set default values - these may be updated later
    pIomLong->tail.ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT;
    pIomLong->tail.ui32SETCLRVal = 0;
#else
    // This is the Pause Boundary for HiPrio transactions
    pIomLong->tail.ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT | ((MAX_INT_BLOCKS % 2) ? IOM_WAIT_FOR_MSPI_BUFFER0 : IOM_WAIT_FOR_MSPI_BUFFER1);
    pIomLong->tail.ui32SETCLRVal = (MAX_INT_BLOCKS % 2) ? IOM_SIGNAL_MSPI_BUFFER0 : IOM_SIGNAL_MSPI_BUFFER1;
#endif
}

// Initialize for each type of transacation
void iom_setup_cq_long(uint32_t ui32Module, uint8_t ui8Priority, uint32_t ui32Dir, uint32_t blockSize, uint32_t ui32SpiChipSelect, uint32_t ui32I2CDevAddr)
{
    am_hal_iom_long_txn_t *pIomLong = &gIomLongTxn;
    uint32_t ui32DMACFGVal;
    ui32DMACFGVal     =
        _VAL2FLD(IOM0_DMACFG_DMAPRI, ui8Priority)     |
        _VAL2FLD(IOM0_DMACFG_DMADIR, ui32Dir == AM_HAL_IOM_TX ? 1 : 0) |
        IOM0_DMACFG_DMAEN_Msk;

    // Command for I2C DEVADDR field in DEVCFG
    pIomLong->head.ui32DEVCFGVal     = _VAL2FLD(IOM0_DEVCFG_DEVADDR, ui32I2CDevAddr);
    //
    // Command to set DMATOTALCOUNT
    //
    pIomLong->head.ui32DMATOTCOUNTVal = blockSize;

    //
    // Command to set DMACFG to start the DMA operation
    //
    pIomLong->tail.ui32DMACFGVal = pIomLong->head.ui32DMACFGVal = ui32DMACFGVal;

    // Initialize the sequence blocks
    for (uint32_t i = 0; i < MAX_INT_BLOCKS; i++)
    {
        uint32_t ui32Cmd;
        //
        // Command to start the transfer.
        //
        ui32Cmd = build_iom_cmd(ui32SpiChipSelect, // ChipSelect
                            ui32Dir,          // ui32Dir
                            true,           // ui32Cont
                            0,           // ui32Offset
                            0,        // ui32OffsetCnt
                            blockSize);  // ui32Bytes

        //
        // Command to set DMATOTALCOUNT
        //
        gIomSequence.block[i].ui32DMATOTCOUNTVal = blockSize;

        //
        // Command to set DMACFG to start the DMA operation
        //
        gIomSequence.block[i].ui32DMACFGVal = ui32DMACFGVal;

        // Command
        gIomSequence.block[i].ui32CMDVal = ui32Cmd;
    }
}

static void
create_iom_mspi_read_transaction(uint32_t blockSize,
                                uint32_t ui32Dir,
                                uint32_t ui32NumBytes,
                                uint32_t ui32Instr, // Address
                                uint32_t ui32InstrLen, // Address length
                                uint32_t ui32SpiChipSelect)
{
    uint32_t ui32Cmd;
    // initialize various fields
    // initialize gIomLongTxn
    // head: ui32OFFSETHIVal, ui32HdrDMATARGADDRVal, ui32HdrCMDVal
    gIomLongTxn.head.ui32OFFSETHIVal   = (uint16_t)(ui32Instr >> 8);

    uint32_t numBlocks = (ui32NumBytes + blockSize - 1) / blockSize;

    // jmp: ui32CQAddrVal
    if (numBlocks > 1)
    {
        ui32Cmd = build_iom_cmd(ui32SpiChipSelect, // ChipSelect
                            ui32Dir,          // ui32Dir
                            true,           // ui32Cont
                            ui32Instr,           // ui32Offset
                            ui32InstrLen,        // ui32OffsetCnt
                            blockSize);  // ui32Bytes
        gIomLongTxn.head.ui32CMDVal   = ui32Cmd;
        if (numBlocks > 2)
        {
            // Identify where in the block array to jump to
            gIomLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gIomSequence.block[MAX_INT_BLOCKS - (numBlocks - 2)];
        }
        else
        {
            // Just jump to tail
            gIomLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gIomLongTxn.tail;
        }
        ui32NumBytes -= blockSize*(numBlocks - 1);
        // tail: ui32HdrCMDVal & ui32DMATOTCOUNTVal
        // Initialize the count & command for tail
        gIomLongTxn.tail.ui32DMATOTCOUNTVal  = ui32NumBytes;
        ui32Cmd = build_iom_cmd(ui32SpiChipSelect, // ChipSelect
                            ui32Dir,          // ui32Dir
                            false,           // ui32Cont
                            0,           // ui32Offset
                            0,        // ui32OffsetCnt
                            ui32NumBytes);  // ui32Bytes
        gIomLongTxn.tail.ui32CMDVal   = ui32Cmd;
    }
    else
    {
        ui32Cmd = build_iom_cmd(ui32SpiChipSelect, // ChipSelect
                            ui32Dir,          // ui32Dir
                            false,           // ui32Cont
                            ui32Instr,           // ui32Offset
                            ui32InstrLen,        // ui32OffsetCnt
                            ui32NumBytes);  // ui32Bytes
        gIomLongTxn.head.ui32CMDVal   = ui32Cmd;
        // Just jump to end
        gIomLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gIomLongTxn.jmpOut;
    }

    // Initialize head - based on how many blocks
    if ((MAX_INT_BLOCKS + 2 - numBlocks) % 2)
    {
        gIomLongTxn.head.ui32DMATARGADDRVal = (uint32_t)&g_TempBuf[1];
        // This is the Pause Boundary for HiPrio transactions
        gIomLongTxn.head.ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT | IOM_WAIT_FOR_MSPI_BUFFER1;
        gIomLongTxn.head.ui32SETCLRVal = IOM_SIGNAL_MSPI_BUFFER1;
    }
    else
    {
        gIomLongTxn.head.ui32DMATARGADDRVal = (uint32_t)&g_TempBuf[0];
        // This is the Pause Boundary for HiPrio transactions
        gIomLongTxn.head.ui32PAUSEENVal = AM_HAL_IOM_CQP_PAUSE_DEFAULT | IOM_WAIT_FOR_MSPI_BUFFER0;
        gIomLongTxn.head.ui32SETCLRVal = IOM_SIGNAL_MSPI_BUFFER0;
    }
}

// MSPI
// One time initialization
void mspi_init_cq_long(uint32_t ui32Module)
{
    am_hal_mspi_long_txn_t *pMspiLong = &gMspiLongTxn;
    // Initialize the head block
    // Initialize the tail block
    pMspiLong->head.ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
    pMspiLong->head.ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
    pMspiLong->head.ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
    pMspiLong->head.ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
    pMspiLong->head.ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
    pMspiLong->head.ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
    pMspiLong->head.ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
    pMspiLong->head.ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;

    pMspiLong->tail.ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
    pMspiLong->tail.ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
    pMspiLong->tail.ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
    pMspiLong->tail.ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
    pMspiLong->tail.ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
    pMspiLong->tail.ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
    pMspiLong->tail.ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
    pMspiLong->tail.ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;

    pMspiLong->head.ui32DMACFG2Val = _VAL2FLD(MSPI_DMACFG_DMAEN, 0);
    pMspiLong->tail.ui32DMACFG2Val = _VAL2FLD(MSPI_DMACFG_DMAEN, 0);

    pMspiLong->tail.ui32DMATARGADDRVal = (MAX_INT_BLOCKS % 2) ? (uint32_t)&g_TempBuf[0] : (uint32_t)&g_TempBuf[1];

    // Initialize the jumps
    gMspiSequence.jmp.ui32CQAddrAddr = pMspiLong->jmpOut.ui32CQAddrAddr = pMspiLong->jmp.ui32CQAddrAddr = (uint32_t)&MSPIn(ui32Module)->CQADDR;
    gMspiSequence.jmp.ui32CQAddrVal = (uint32_t)&pMspiLong->tail;
    // Initialize the sequence blocks
    for (uint32_t i = 0; i < MAX_INT_BLOCKS; i++)
    {
        gMspiSequence.block[i].ui32PAUSENAddr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        gMspiSequence.block[i].ui32PAUSEN2Addr = (uint32_t)&MSPIn(ui32Module)->CQPAUSE;
        gMspiSequence.block[i].ui32DMATARGADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMATARGADDR;
        gMspiSequence.block[i].ui32DMATOTCOUNTAddr = (uint32_t)&MSPIn(ui32Module)->DMATOTCOUNT;
        gMspiSequence.block[i].ui32DMACFG1Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        gMspiSequence.block[i].ui32DMACFG2Addr = (uint32_t)&MSPIn(ui32Module)->DMACFG;
        gMspiSequence.block[i].ui32SETCLRAddr = (uint32_t)&MSPIn(ui32Module)->CQSETCLEAR;
        gMspiSequence.block[i].ui32DMADEVADDRAddr = (uint32_t)&MSPIn(ui32Module)->DMADEVADDR;
        // Pause Conditions
#if 0
        // Normal Memory
        // Set default values - these may be updated later
        gMspiSequence.block[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT;
        gMspiSequence.block[i].ui32SETCLRVal = 0;
#else
        // This is the Pause Boundary for HiPrio transactions
        gMspiSequence.block[i].ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((i % 2) ? MSPI_WAIT_FOR_IOM_BUFFER0 : MSPI_WAIT_FOR_IOM_BUFFER1);
        gMspiSequence.block[i].ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
        gMspiSequence.block[i].ui32SETCLRVal = (i % 2) ? MSPI_SIGNAL_IOM_BUFFER0 : MSPI_SIGNAL_IOM_BUFFER1;
#endif
        gMspiSequence.block[i].ui32DMATARGADDRVal = (i % 2) ? (uint32_t)&g_TempBuf[0] : (uint32_t)&g_TempBuf[1];
        gMspiSequence.block[i].ui32DMACFG2Val = _VAL2FLD(MSPI_DMACFG_DMAEN, 0);
    }

    pMspiLong->tail.ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
    pMspiLong->head.ui32PAUSEEN2Val = AM_HAL_MSPI_PAUSE_DEFAULT;
#if 0
    // Normal Memory
    // Set default values - these may be updated later
    pMspiLong->tail.ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT;
    pMspiLong->tail.ui32SETCLRVal = 0;
#else
    // This is the Pause Boundary for HiPrio transactions
    pMspiLong->tail.ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | ((MAX_INT_BLOCKS % 2) ? MSPI_WAIT_FOR_IOM_BUFFER0 : MSPI_WAIT_FOR_IOM_BUFFER1);
    pMspiLong->tail.ui32SETCLRVal = (MAX_INT_BLOCKS % 2) ? MSPI_SIGNAL_IOM_BUFFER0 : MSPI_SIGNAL_IOM_BUFFER1;
#endif
}

// Initialize for each type of transacation
void mspi_setup_cq_long(uint32_t ui32Module, uint8_t ui8Priority, am_hal_mspi_dir_e eDirection, uint32_t blockSize)
{
    am_hal_mspi_long_txn_t *pMspiLong = &gMspiLongTxn;
    uint32_t ui32DmaCfg =
        _VAL2FLD(MSPI_DMACFG_DMAPWROFF, 0)   |  // DMA Auto Power-off not supported!
        _VAL2FLD(MSPI_DMACFG_DMAPRI, ui8Priority)    |
        _VAL2FLD(MSPI_DMACFG_DMADIR, eDirection)     |
        _VAL2FLD(MSPI_DMACFG_DMAEN, 3);

    pMspiLong->head.ui32DMATOTCOUNTVal = blockSize;
    pMspiLong->head.ui32DMACFG1Val     = pMspiLong->tail.ui32DMACFG1Val = ui32DmaCfg;

    // Initialize the sequence blocks
    for (uint32_t i = 0; i < MAX_INT_BLOCKS; i++)
    {
        gMspiSequence.block[i].ui32DMACFG1Val = ui32DmaCfg;
        gMspiSequence.block[i].ui32DMATOTCOUNTVal = blockSize;
    }
}

static void
create_mspi_iom_write_transaction(uint32_t blockSize,
                                am_hal_mspi_dir_e eDirection,
                                uint32_t ui32DevAddr,
                                uint32_t ui32NumBytes)
{
    am_hal_mspi_long_txn_t *pMspiLong = &gMspiLongTxn;
    uint32_t numBlocks = (ui32NumBytes + blockSize - 1) / blockSize;

    pMspiLong->head.ui32DMADEVADDRVal = ui32DevAddr;
    // jmp: ui32CQAddrVal
    if (numBlocks > 1)
    {
        if (numBlocks > 2)
        {
            // Identify where in the block array to jump to
            gMspiLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gMspiSequence.block[MAX_INT_BLOCKS - (numBlocks - 2)];
            for (uint32_t i = 0; i < (numBlocks - 2); i++)
            {
                gMspiSequence.block[MAX_INT_BLOCKS - (numBlocks - 2) + i].ui32DMADEVADDRVal = ui32DevAddr + (i + 1) * blockSize;
            }
        }
        else
        {
            // Just jump to tail
            gMspiLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gMspiLongTxn.tail;
        }
        ui32NumBytes -= blockSize*(numBlocks - 1);
        // tail: ui32HdrCMDVal & ui32DMATOTCOUNTVal
        // Initialize the count & command for tail
        gMspiLongTxn.tail.ui32DMADEVADDRVal = ui32DevAddr + (numBlocks-1)*blockSize;
        gMspiLongTxn.tail.ui32DMATOTCOUNTVal  = ui32NumBytes;
    }
    else
    {
        // Just jump to end
        gMspiLongTxn.jmp.ui32CQAddrVal = (uint32_t)&gMspiLongTxn.jmpOut;
    }

    // Initialize head - based on how many blocks
    if ((MAX_INT_BLOCKS + 2 - numBlocks) % 2)
    {
        gMspiLongTxn.head.ui32DMATARGADDRVal = (uint32_t)&g_TempBuf[1];
        // This is the Pause Boundary for HiPrio transactions
        gMspiLongTxn.head.ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | MSPI_WAIT_FOR_IOM_BUFFER1;
        gMspiLongTxn.head.ui32SETCLRVal = MSPI_SIGNAL_IOM_BUFFER1;
    }
    else
    {
        gMspiLongTxn.head.ui32DMATARGADDRVal = (uint32_t)&g_TempBuf[0];
        // This is the Pause Boundary for HiPrio transactions
        gMspiLongTxn.head.ui32PAUSEENVal = AM_HAL_MSPI_CQP_PAUSE_DEFAULT | MSPI_WAIT_FOR_IOM_BUFFER0;
        gMspiLongTxn.head.ui32SETCLRVal = MSPI_SIGNAL_IOM_BUFFER0;
    }
}

#endif

uint32_t
init_mspi_iom_xfer(void)
{
    uint32_t      ui32Status = 0;
    uint32_t u32Arg;

    //
    // Clear flags
    //
    u32Arg = 0x003F0000;  // clear all flags
    am_hal_iom_control(g_IOMHandle, AM_HAL_IOM_REQ_FLAG_SETCLR, &u32Arg);
#if defined(SEQLOOP) && defined(RUN_AUTONOMOUS)
    u32Arg = 0x003B0004;  // set flag PAUSEFLAG, clear all other flags
#endif
    am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_FLAG_SETCLR, &u32Arg);

    //
    // Link MSPI and IOM
    //
    u32Arg = FRAM_IOM_MODULE;
    ui32Status = am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_LINK_IOM, &u32Arg);
    if (ui32Status)
    {
        return ui32Status;
    }
#ifdef CQ_RAW
    iom_init_cq_long(FRAM_IOM_MODULE);
    iom_setup_cq_long(FRAM_IOM_MODULE, 1, AM_HAL_IOM_TX, SPI_TXN_SIZE, g_FramChipSelect[FRAM_IOM_MODULE], 0);
    mspi_init_cq_long(0);
    mspi_setup_cq_long(0, 1, AM_HAL_MSPI_RX, SPI_TXN_SIZE);
#endif
    return ui32Status;
}

int
start_mspi_iom_xfer(void)
{
    uint32_t      ui32Status = 0;
    am_hal_iom_callback_t   iomCb = 0;
    am_hal_mspi_callback_t  mspiCb = 0;

#ifdef SEQLOOP
    bool          bBool;
#endif
    DEBUG_GPIO_HIGH(TEST_GPIO);
#ifndef RUN_AUTONOMOUS
    iomCb = fram_write_complete;
    mspiCb = flash_read_complete;
#endif


    DEBUG_PRINT("\nInitiating MSP -> IOM Transfer\n");
#ifdef SEQLOOP
    //
    // Set in Sequence mode
    //
    bBool = true;
    am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_SET_SEQMODE, &bBool);
    am_hal_iom_control(g_IOMHandle, AM_HAL_IOM_REQ_SET_SEQMODE, &bBool);
#endif

    //
    // Queue up FRAM Writes and Flash Reads
    //
    for (uint32_t address = 0, bufIdx = 0; address < (BLOCK_SIZE - VARIABLE_SIZE_CHANGE*numIter); address += SPI_TXN_SIZE, bufIdx++)
    {
#ifndef CQ_RAW
        uint32_t bufOdd = (bufIdx + (numIter * VARIABLE_SIZE_CHANGE) / SPI_TXN_SIZE) % 2;
        ui32Status = flash_func.flash_read_adv(g_FlashHdl, (uint8_t *)g_TempBuf[bufOdd], address + VARIABLE_SIZE_CHANGE*numIter,
                                       (((address + SPI_TXN_SIZE) >= BLOCK_SIZE) ? (BLOCK_SIZE - address) : SPI_TXN_SIZE),
                                       (bufOdd ? MSPI_WAIT_FOR_IOM_BUFFER1 : MSPI_WAIT_FOR_IOM_BUFFER0),
                                       (bufOdd ? MSPI_SIGNAL_IOM_BUFFER1 : MSPI_SIGNAL_IOM_BUFFER0),
                                       (((address + SPI_TXN_SIZE) >= (BLOCK_SIZE - VARIABLE_SIZE_CHANGE*numIter)) ? mspiCb : 0),
                                       0);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up MSPI Read transaction\n");
            break;
        }
        ui32Status = fram_func.fram_nonblocking_write_adv(g_IomDevHdl, (uint8_t *)g_TempBuf[bufOdd], address,
                                       (((address + SPI_TXN_SIZE) >= BLOCK_SIZE) ? (BLOCK_SIZE - address) : SPI_TXN_SIZE),
                                       (bufOdd ? IOM_WAIT_FOR_MSPI_BUFFER1 : IOM_WAIT_FOR_MSPI_BUFFER0),
                                       (bufOdd ? IOM_SIGNAL_MSPI_BUFFER1 : IOM_SIGNAL_MSPI_BUFFER0),
                                       (((address + SPI_TXN_SIZE) >= BLOCK_SIZE) ? iomCb : 0),
                                       0);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up SPI Write transaction\n");
             break;
        }
#endif
    }

    if (ui32Status == 0)
    {
#ifdef CQ_RAW
        // Queue up the CQ Raw
        am_hal_cmdq_entry_t jump;
        // MSPI
        create_mspi_iom_write_transaction(SPI_TXN_SIZE,
                                        AM_HAL_MSPI_RX,
                                        VARIABLE_SIZE_CHANGE*numIter,
                                        BLOCK_SIZE - VARIABLE_SIZE_CHANGE*numIter);
        jump.address = (uint32_t)&MSPIn(0)->CQADDR;
        jump.value = (uint32_t)&gMspiLongTxn;

        am_hal_mspi_cq_raw_t rawMspiCfg;
        rawMspiCfg.ui32PauseCondition = MSPI_WAIT_FOR_IOM_BUFFER0;
        rawMspiCfg.ui32StatusSetClr = 0;
        rawMspiCfg.pCQEntry = &jump;
        rawMspiCfg.numEntries = sizeof(am_hal_cmdq_entry_t) / 8;
        rawMspiCfg.pfnCallback = mspiCb;
        rawMspiCfg.pCallbackCtxt = 0;
        rawMspiCfg.pJmpAddr = &gMspiLongTxn.jmpOut.ui32CQAddrVal;
        ui32Status = am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_CQ_RAW, &rawMspiCfg);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up MSPI Read transaction\n");
            while(1);
        }
        // IOM
        create_iom_mspi_read_transaction(SPI_TXN_SIZE,
                                        AM_HAL_IOM_TX,
                                        BLOCK_SIZE - VARIABLE_SIZE_CHANGE*numIter,
                                        0, // address
                                        3, // TODO Address size - dependent on FRAM device - should be abstracted out in fram_device_func_t
                                        g_FramChipSelect[FRAM_IOM_MODULE]);
        DEBUG_GPIO_HIGH(TEST_GPIO1);
        // Configure FRAM for writing
        ui32Status = fram_func.fram_nonblocking_write_adv(g_IomDevHdl, NULL, 0, 0,
                                       IOM_WAIT_FOR_MSPI_BUFFER0,
                                       0,
                                       0,
                                       0);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up SPI Write transaction\n");
            while(1);
        }
        DEBUG_GPIO_LOW(TEST_GPIO1);

        //
        // Queue up the CQ Raw
        //
        jump.address = (uint32_t)&IOMn(FRAM_IOM_MODULE)->CQADDR;
        jump.value = (uint32_t)&gIomLongTxn;

        am_hal_iom_cq_raw_t rawIomCfg;
        rawIomCfg.ui32PauseCondition = IOM_WAIT_FOR_MSPI_BUFFER0;
        rawIomCfg.ui32StatusSetClr = 0;
        rawIomCfg.pCQEntry = &jump;
        rawIomCfg.numEntries = sizeof(am_hal_cmdq_entry_t) / 8;
        rawIomCfg.pfnCallback = iomCb;
        rawIomCfg.pCallbackCtxt = 0;
        rawIomCfg.pJmpAddr = &gIomLongTxn.jmpOut.ui32CQAddrVal;
        ui32Status = am_hal_iom_control(g_IOMHandle, AM_HAL_IOM_REQ_CQ_RAW, &rawIomCfg);
        if (ui32Status)
        {
            DEBUG_PRINT("\nFailed to queue up SPI Write transaction\n");
            while(1);
        }

#endif
#ifdef SEQLOOP
        // End the sequence and start
        am_hal_iom_seq_end_t iomLoop;
        am_hal_mspi_seq_end_t mspiLoop;
        iomLoop.bLoop = true;
        // Let IOM be fully controlled by MSPI
        iomLoop.ui32PauseCondition = iomLoop.ui32StatusSetClr = 0;
        am_hal_iom_control(g_IOMHandle, AM_HAL_IOM_REQ_SEQ_END, &iomLoop);
        mspiLoop.bLoop = true;
#ifndef RUN_AUTONOMOUS
        // Let MSPI be controlled by a flag
        mspiLoop.ui32PauseCondition = PAUSEFLAG;
        mspiLoop.ui32StatusSetClr = PAUSEFLAG << 16;
#else
        mspiLoop.ui32PauseCondition = 0;
        mspiLoop.ui32StatusSetClr = 0;
#endif
        am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_SEQ_END, &mspiLoop);
#endif
    }
    else
    {
        while(1);
    }
    DEBUG_GPIO_LOW(TEST_GPIO);
    return ui32Status;
}

int
init_fram_data(void)
{
    uint32_t      ui32Status;

    // Verify FRAM data
    for (uint32_t address = 0; address < (BLOCK_SIZE); address += SPI_TXN_SIZE)
    {
        uint32_t numBytes = (((address + SPI_TXN_SIZE) >= BLOCK_SIZE) ? (BLOCK_SIZE - address) : SPI_TXN_SIZE);
        //
        // Generate data into the Sector Buffer
        //
        for (uint32_t i = 0; i < numBytes / 4; i++)
        {
            g_TempBuf[0][i] = 0xFF; //address + i*4;
        }

        g_bDone = 0;
        //
        // Write the TX buffer into the target sector.
        //
        DEBUG_PRINT("Writing %d Bytes to Address 0x%x\n", numBytes, address);
        ui32Status = fram_func.fram_nonblocking_write_adv(g_IomDevHdl, (uint8_t *)g_TempBuf[0], address, numBytes,
                                       0,
                                       0,
                                       fram_write_complete,
                                       0);
        if (ui32Status)
        {
            DEBUG_PRINT("Failed to write FRAM!\n");
             return -1;
        }
        while (!g_bDone);
    }
    // Wait for writes to finish
    for (uint32_t address = 0; address < (BLOCK_SIZE); address += SPI_TXN_SIZE)
    {
        uint32_t numBytes = (((address + SPI_TXN_SIZE) >= BLOCK_SIZE) ? (BLOCK_SIZE - address) : SPI_TXN_SIZE);
        //
        // Read the data back into the RX buffer.
        //
        DEBUG_PRINT("Read %d Bytes from Address 0x%x\n", numBytes, address);
        // Initiate read of a block of data from FRAM
        ui32Status = fram_func.fram_blocking_read(g_IomDevHdl, (uint8_t *)&g_TempBuf[1], address, numBytes);

        if (0 != ui32Status)
        {
            DEBUG_PRINT("Failed to read FRAM!\n");
            return -1;
        }

        //
        // Compare the buffers
        //
        DEBUG_PRINT("Comparing the TX and RX Buffers\n");
        for (uint32_t i = 0; i < numBytes / 4; i++)
        {
            if (g_TempBuf[1][i] != 0xFF) // (address + i*4))
            {
                DEBUG_PRINT("TX and RX buffers failed to compare!\n");
                return -1;
            }
        }
    }
    return 0;
}


#ifdef VERIFY_DATA
int
verify_fram_data(void)
{
    uint32_t      ui32Status;

    // Verify FRAM data
    for (uint32_t address = 0; address < (BLOCK_SIZE - numIter*VARIABLE_SIZE_CHANGE); address += SPI_TXN_SIZE)
    {
        uint32_t numBytes = (((address + SPI_TXN_SIZE) >= (BLOCK_SIZE - numIter*VARIABLE_SIZE_CHANGE)) ? (BLOCK_SIZE - numIter*VARIABLE_SIZE_CHANGE - address) : SPI_TXN_SIZE);
        //
        // Read the data back into the RX buffer.
        //
        // Initiate read of a block of data from FRAM
        ui32Status = fram_func.fram_blocking_read(g_IomDevHdl, (uint8_t *)&g_TempBuf[1], address, numBytes);

        if (0 != ui32Status)
        {
            DEBUG_PRINT("Failed to read FRAM!\n");
            return -1;
        }

        //
        // Compare the buffers
        //
        for (uint32_t i = 0; i < numBytes / 4; i++)
        {
            if (g_TempBuf[1][i] != address + numIter*VARIABLE_SIZE_CHANGE + i*4)
            {
                DEBUG_PRINT("TX and RX buffers failed to compare!\n");
                return -1;
            }
        }
    }
    DEBUG_PRINT("FRAM data matches the expectation\n");
    return 0;
}
#endif

//*****************************************************************************
//
// MSPI Example Main.
//
//*****************************************************************************
int
main(void)
{
    uint32_t      ui32Status;
    int iRet;
    am_devices_mspi_flash_config_t mspiFlashCfg = MSPI_Flash_Config;

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

#if 0
    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();
#else
    //
    // Initialize the ITM printf interface.
    //
    am_bsp_itm_printf_enable();
#endif
    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Apollo3 MSPI-IOM transfer Example\n\n");

    // GPIO used to track the CPU sleeping
    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_enable(CPU_SLEEP_GPIO);
    am_hal_gpio_fastgpio_clr(CPU_SLEEP_GPIO);
    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_disable(CPU_SLEEP_GPIO);
    am_hal_gpio_fastgpio_clr(CPU_SLEEP_GPIO);
    AM_HAL_GPIO_MASKCREATE(sGpioIntMaskCpu);
    am_hal_gpio_fast_pinconfig(AM_HAL_GPIO_MASKBIT(psGpioIntMaskCpu, CPU_SLEEP_GPIO),
                               g_AM_HAL_GPIO_OUTPUT, 0);

    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_enable(TEST_GPIO);
    am_hal_gpio_fastgpio_clr(TEST_GPIO);

    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_disable(TEST_GPIO);
    am_hal_gpio_fastgpio_clr(TEST_GPIO);
    AM_HAL_GPIO_MASKCREATE(sGpioIntMaskTest);
    am_hal_gpio_fast_pinconfig(AM_HAL_GPIO_MASKBIT(psGpioIntMaskTest, TEST_GPIO),
                               g_AM_HAL_GPIO_OUTPUT, 0);

    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_enable(TEST_GPIO1);
    am_hal_gpio_fastgpio_clr(TEST_GPIO1);

    //
    // Configure the pins that are to be used for Fast GPIO.
    //
    am_hal_gpio_fastgpio_disable(TEST_GPIO1);
    am_hal_gpio_fastgpio_clr(TEST_GPIO1);
    AM_HAL_GPIO_MASKCREATE(sGpioIntMaskTest1);
    am_hal_gpio_fast_pinconfig(AM_HAL_GPIO_MASKBIT(psGpioIntMaskTest1, TEST_GPIO1),
                               g_AM_HAL_GPIO_OUTPUT, 0);

#if FIREBALL_CARD || FIREBALL2_CARD
    iRet = fireball_init();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize Fireball card\n");
        while(1);
    }

#endif

    //
    // Initialize the MSPI Flash
    //
    iRet = mspi_flash_init(mspiFlashCfg);
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize MSPI Flash\n");
        while(1);
    }

    //
    // Initialize the IOM FRAM
    //
    iRet = fram_init();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize FRAM\n");
        while(1);
    }

    //
    // Initialize FRAM Data
    //
    iRet = init_fram_data();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize FRAM data\n");
        while(1);
    }

#ifdef VERIFY_DATA
    //
    // Initialize Flash Data
    //
    iRet = init_mspi_flash_data();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize MSPI Flash data\n");
        while(1);
    }
#endif

    am_hal_interrupt_master_enable();

    iRet = init_mspi_iom_xfer();
    if (iRet)
    {
        DEBUG_PRINT("Unable to initialize MSPI IOM transfer\n");
        while(1);
    }

    g_bDone = 0;
#if defined(SEQLOOP) && defined(RUN_AUTONOMOUS)
    timerA0_init();
#endif
    iRet = start_mspi_iom_xfer();
    if (iRet)
    {
        DEBUG_PRINT("Unable to start MSPI IOM transfer\n");
        while(1);
    }
//    numIter++;

    DEBUG_PRINT("Getting into Wait Loop\n");

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Disable interrupt while we decide whether we're going to sleep.
        //
        uint32_t ui32IntStatus = am_hal_interrupt_master_disable();

        if (!g_bDone)
        {
            DEBUG_GPIO_HIGH(CPU_SLEEP_GPIO);
            am_hal_sysctrl_sleep(true);
            DEBUG_GPIO_LOW(CPU_SLEEP_GPIO);

            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
        }
        else if (g_bDone)
        {
            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
            g_bDone = false;
#ifdef VERIFY_DATA
            if (verify_fram_data())
            {
                DEBUG_PRINT("Verify data failed!\n");
                while(1);
            }
#endif
            if (numIter++ < NUM_ITERATIONS)
            {
#ifndef SEQLOOP
                iRet = start_mspi_iom_xfer();
#else
                uint32_t flag = PAUSEFLAG;
                iRet = am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_FLAG_SETCLR, &flag);
#endif
                if (iRet)
                {
                    DEBUG_PRINT("Unable to start MSPI IOM transfer\n");
                    while(1);
                }
            }
            else
            {
                break;
            }
        }
    }

#ifdef SEQLOOP
    // Set in Sequence mode
    bool bBool = false;
    am_hal_mspi_control(g_MSPIHdl, AM_HAL_MSPI_REQ_SET_SEQMODE, &bBool);
    am_hal_iom_control(g_IOMHandle, AM_HAL_IOM_REQ_SET_SEQMODE, &bBool);
#endif
    //
    // Clean up the FRAM before exit.
    //
    iRet = fram_deinit();
    if (iRet)
    {
        DEBUG_PRINT("Unable to terminate FRAM\n");
    }
    am_hal_interrupt_master_disable();
    NVIC_DisableIRQ(mspi_interrupts[MSPI_TEST_MODULE]);
    //
    // Clean up the MSPI before exit.
    //
    ui32Status = flash_func.flash_term(g_FlashHdl);
    if (AM_DEVICES_MSPI_FLASH_STATUS_SUCCESS != ui32Status)
    {
        DEBUG_PRINT("Failed to shutdown the MSPI and Flash Device!\n");
    }

    //
    //  End banner.
    //
    am_util_stdio_printf("Apollo3 MSPI-IOM Transfer Example Complete\n");

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
