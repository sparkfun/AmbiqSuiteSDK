Name:
=====
 multi_boot


Description:
============
 Bootloader program accepting multiple host protocols.


Multiboot is a bootloader program that supports flash programming over UART,
SPI, and I2C. The correct protocol is selected automatically at boot time.
The messaging is expected to follow little-endian format, which is native to
the Cortex M4 used in Apollo and Apollo2.

If a valid image is already present on the target device, Multiboot will run
that image.  Otherwise it will wait for a new image to be downloaded from
the host.  A new image download can be forced on the target by using the
"override" capability via one of the pins.

Running this example requires 2 EVBs - one EVB to run multi_boot, the second
to run a host example such as spi_boot_host or i2c_boot_host. The two EVBs
are generally fly wired together as shown below.

The most straightforward method of running the demonstration is to use two
EVBs of the same type (i.e. 2 Apollo EVBs or 2 Apollo2 EVBs) as both the
host and the target.  Then the pins on the two EVBs are connected as shown
in the chart including the optional reset and override pins.  With these
connections, the host controls everything and no user intervention is
required other than the press the reset button on the host to initiate the
process.

The host downloads an executable (target) image to the slave that will run
on the target.  By default that image is binary_counter, which is obvious
to see running on the slave.

In the default scenario (two boards of the same type), the image downloaded
by the host is compatible with the host board type, so it will run without
modification on the target.  In the case of the host device being different
from the target, the image downloaded from the host must be modified to be
compatible with the target (requires rebuilding the host example).

The EVB Button1 (usually labelled BTN2 on the EVB silkscreen) is the manual
override which can be used to force downloading of a new image even if a
valid image already exists on the target. The host examples use the same
"override" pin signal when downloading a new image.

PIN fly lead connections assumed by multi_boot:
HOST                                    SLAVE (multi_boot target)
--------                                --------
GPIO[2]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
GPIO[4]  OVERRIDE pin   (host to slave) GPIO[18] Override pin or n/c
GPIO[5]  IOM0 SPI CLK/I2C SCL           GPIO[0]  IOS SPI SCK/I2C SCL
GPIO[6]  IOM0 SPI MISO/I2C SDA          GPIO[1]  IOS SPI MISO/I2C SDA
GPIO[7]  IOM0 SPI MOSI                  GPIO[2]  IOS SPI MOSI
GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
GPIO[17] Slave reset (host to slave)    Reset Pin (NRST) or n/c
GND                                     GND
Reset and Override pin connections from Host are optional
Keeping Button1 pressed on target has same effect as host driving override


******************************************************************************


