Name:
=====
 iom_fram


Description:
============
 Example that demonstrates IOM, connecting to a SPI or I2C FRAM

Purpose:
========
FRAM is initialized with a known pattern data using Blocking IOM Write.
This example starts a 1 second timer. At each 1 second period, it initiates
reading a fixed size block from the FRAM device using Non-Blocking IOM
Read, and comparing against the predefined pattern

Printing takes place over the ITM at 1M Baud.

Additional Information:
=======================
Define USE_SPI to select SPI or I2C
Define one of FRAM_DEVICE_ macros to select the FRAM device
And if the fireball device card is used, this example can work on:
Apollo3_eb + Fireball
Apollo3_eb + Fireball2
Recommend to use 1.8V power supply voltage.
Define FIREBALL_CARD or FIREBALL2_CARD in the config-template.ini file to select.



******************************************************************************


