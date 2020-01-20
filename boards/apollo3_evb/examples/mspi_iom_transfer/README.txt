Name:
=====
 mspi_iom_xfer


Description:
============
 Example demonstrating the hardware assisted MSPI to IOM transfer


Purpose:
========
This example demonstrates transferring a large buffer from a flash device
connected on MSPI, to a FRAM device connected to IOM, using hardware
handshaking in Apollo3 - with minimal CPU involvement.

At initialization, both the FRAM and Flash are initialized and a set pattern
data is written to the flash for transfer to FRAM.

The FRAM is connected to IOM using SPI interface, and hence the transactions
are limited in size to 4095 Bytes.
The program here creates a command queue for both the MSPI and IOM, to
create a sequence of transactions - each reading a segment of the source
buffer to a temp buffer in internal SRAM, and then writing the same to the
FRAM using the IOM. It uses hardware handshaking so that the IOM transaction
is started only once the segement is read out completely from MSPI Flash.

To best utilize the buses, a ping-pong model is used using two temporary
buffers in SRAM. This allows the interfaces to not idle while waiting for
other to finish - essentially achieving close to the bandwidth achieved by
the slower of the two.

Printing takes place over the ITM at 1M Baud.

Additional Information:
=======================
Configurable parameters at compile time:
IOM to use (FRAM_IOM_MODULE)
FRAM device to use (define one of FRAM_DEVICE_* to 1)
MSPI Flash to use - uses compile time definitions from am_device_mspi_flash.h
BLOCK_SIZE - total size of transaction
SPI_TXN_SIZE - size of temporary ping-pong buffer
CPU_SLEEP_GPIO - tracks CPU sleeping on analyzer
VERIFY_DATA - Enables reading back of the data from IOM to check & verify the accuracy
This can only be enabled if SEQLOOP is not being used

Operating modes:
SEQLOOP not defined - The CQ is programmed each iteration
SEQLOOP - Advanced mode, to create sequence once, which repeats when triggered by callback at the end of each iteration
SEQLOOP - RUN_AUTONOMOUS - Sequence created once, and it repeats indefintely till paused (as a result of timer)
CQ_RAW - Uses Preconstructed CQ for IOM and MSPI - to save on the time to program the same at run time

Best way to see the example in action is to connect logic analyzer and monitor the signals
Apart from the IO signals below, one can also monitor CPU_SLEEP_GPIO to monitor CPU in deep sleep
Pin connections:
IOM:
Particular IOM to use for this example is controlled by macro FRAM_IOM_MODULE
This example use apollo3_eb board connected to a fireball
Fireball is populated with MB85RS1MT SPI FRAM devices on each of the IOM's
with respective pin definitions in the BSP
Default pin settings for this example using IOM1 are:
#define AM_BSP_GPIO_IOM1_CS             14
#define AM_BSP_IOM1_CS_CHNL             2
#define AM_BSP_GPIO_IOM1_MISO           9
#define AM_BSP_GPIO_IOM1_MOSI           10
#define AM_BSP_GPIO_IOM1_SCK            8

MSPI:
The MSPI flash device uses is controlled by macro FRAM_DEVICE_* (set one of them to 1)
This example uses apollo3_eb board connected to a fireball
Fireball is populated with CYPRESS_S25FS064S flash on CE0
#define AM_BSP_GPIO_MSPI_CE0            19
#define AM_BSP_MSPI_CE0_CHNL            0
#define AM_BSP_GPIO_MSPI_D0             22
#define AM_BSP_GPIO_MSPI_D1             26
#define AM_BSP_GPIO_MSPI_D2             4
#define AM_BSP_GPIO_MSPI_D3             23
#define AM_BSP_GPIO_MSPI_SCK            24

And if the fireball device card is used, this example can work on:
Apollo3_eb + Fireball
Apollo3_eb + Fireball2
Recommend to use 1.8V power supply voltage.
Define FIREBALL_CARD or FIREBALL2_CARD in the config-template.ini file to select.
Define CYPRESS_S25FS064S or ADESTO_ATXP032 for Fireball
Define ADESTO_ATXP032 for Fireball2



******************************************************************************


