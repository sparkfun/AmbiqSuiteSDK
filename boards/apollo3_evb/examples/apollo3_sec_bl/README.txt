Name:
=====
 apollo3_secbl


Description:
============
 A simple secondary bootloader program example for Apollo3


Purpose:
========
This program is an example template for a secondary bootloader program for Apollo3.
It demonstrates how to access info0 key area. It demonstrates how to use the Ambiq SBL OTA 
framework for customer specific OTAs, e.g. to support external flash, or to support more 
advanced auth/enc schemes. It demonstrates how to validate & transfer control to the real 
main program image (assumed to be at address specified by MAIN_PROGRAM_ADDR_IN_FLASH in flash)
after locking the info0 area before exiting

Printing takes place over the ITM at 1M Baud.

Additional Information:
=======================
To exercise this program:
Flash the main program at 0x10000 (MAIN_PROGRAM_ADDR_IN_FLASH)
Link this program at the address suitable for SBL nonsecure (0xC000) or secure (0xC100)
configuration
To test OTA - construct images using magic numbers in the range matching
AM_IMAGE_MAGIC_CUST
To test INFO0 key area access - need to keep INFO0->Security->PLONEXIT as 0



******************************************************************************


