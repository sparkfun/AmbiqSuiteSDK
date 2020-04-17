Name:
=====
 uart_boot_host


Description:
============
 Converts UART Wired transfer commands to SPI for use with SBL SPI testing.


Purpose:
========
This example running on an intermediate board, along with the standard
uart_wired_update script running on host PC, can be used as a way to
communicate to Apollo3 SBL using SPI mode.

Printing takes place over the ITM at 1M Baud.

Additional Information:
=======================
PIN fly lead connections assumed:
HOST (this board)                       SLAVE (Apollo3 SBL target)
--------                                --------
Apollo3 SPI or I2C common connections:
GPIO[2]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
GPIO[4]  OVERRIDE pin   (host to slave) GPIO[16] Override pin or n/c
GPIO[17] Slave reset (host to slave)    Reset Pin or n/c
GND                                     GND

Apollo3 SPI additional connections:
GPIO[5]  IOM0 SPI CLK                   GPIO[0]  IOS SPI SCK
GPIO[6]  IOM0 SPI MISO                  GPIO[2]  IOS SPI MISO
GPIO[7]  IOM0 SPI MOSI                  GPIO[1]  IOS SPI MOSI
GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE

Apollo3 I2C additional connections:
GPIO[5]  I2C SCL                        GPIO[0]  I2C SCL
GPIO[6]  I2C SDA                        GPIO[1]  I2C SDA

Reset and Override pin connections from Host are optional, but using them
automates the entire process.

SPI or I2C mode can be handled in a couple of ways:
- SPI mode is the default (i.e. don't press buttons or tie pins low).
- For I2C, press button2 during reset and hold it until the program begins,
i.e. you see the "I2C clock = " msg.
Alternatively the button2 pin can be tied low.
- Note that on the Apollo3 EVB, button2 is labelled as 'BTN4', which is
the button located nearest the end of the board.
Also on the Apollo3 EVB, BTN4 uses pin 19.  It happens that the header
pin for pin 19 on the EVB is adjacent to a ground pin, so a jumper can
be used to assert I2C mode.



******************************************************************************


