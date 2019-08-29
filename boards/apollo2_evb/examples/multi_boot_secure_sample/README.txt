Name:
=====
 multi_boot


Description:
============
 Bootloader program accepting multiple host protocols.


This is a bootloader program that supports flash programming over UART,
SPI, and I2C. The correct protocol is selected automatically at boot time.
The messaging is expected to follow little-endian format, which is native to
Apollo1/2.
This version of bootloader also supports security features -
Image confidentiality, Authentication, and key revocation is supported
using a simple CRC-CBC based encryption mechanism.

Default override pin corresponds to Button1. So, even if a valid image is
present in flash, bootloader can be forced to wait for new image from host.

PIN fly lead connections assumed by multi_boot:
HOST                                    SLAVE (multi_boot target)
--------                                --------
GPIO[2]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
GPIO[4]  OVERRIDE pin   (host to slave) GPIO[18] Override pin or n/c
GPIO[5]  IOM0 SPI CLK/I2C SCL           GPIO[0]  IOS SPI SCK/I2C SCL
GPIO[6]  IOM0 SPI MISO/I2C SDA          GPIO[1]  IOS SPI MISO/I2C SDA
GPIO[7]  IOM0 SPI MOSI                  GPIO[2]  IOS SPI MOSI
GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
GPIO[17] Slave reset (host to slave)    Reset Pin or n/c
Reset and Override pin connections from Host are optional
Keeping Button1 pressed on target has same effect as host driving override


******************************************************************************


