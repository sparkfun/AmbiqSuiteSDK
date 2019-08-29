Name:
=====
 i2c_boot_host


Description:
============
 An example to drive the IO Slave on a second board.


This example acts as the boot host for spi_boot and multi_boot on Apollo
and Apollo2 MCUs. It will deliver a predefined firmware image to a boot
slave over an I2C protocol. The purpose of this demo is to show how a host
processor might store, load, and update the firmware on an Apollo or
Apollo2 device that is connected as a slave.

Please see the multi_boot README.txt for more details on how to run the
examples.

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
GND                                     GND
Reset and Override pin connections from Host are optional
Keeping Button1 pressed on target has same effect as host driving override


******************************************************************************


