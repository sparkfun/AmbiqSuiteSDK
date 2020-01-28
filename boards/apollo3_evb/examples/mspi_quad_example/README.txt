Name:
=====
 mspi_quad_example


Description:
============
 Example of the MSPI operation with Quad SPI Flash.


Purpose:
========
This example configures an MSPI connected flash device in Quad mode
and performs various operations - verifying the correctness of the same
Operations include - Erase, Write, Read, Read using XIP Apperture and XIP.

Printing takes place over the ITM at 1M Baud.

Additional Information:
=======================
If the fireball device card is used, this example can work on:
Apollo3_eb + Fireball
Apollo3_eb + Fireball2
Recommend to use 1.8V power supply voltage.
Define FIREBALL_CARD in the config-template.ini file to select.
Define CYPRESS_S25FS064S or ADESTO_ATXP032 for Fireball
Define ADESTO_ATXP032 for Fireball2



******************************************************************************


