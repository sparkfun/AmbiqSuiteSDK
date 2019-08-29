Name:
=====
 bc_boot_demo


Description:
============
 Example that displays the timer count on the LEDs.


This example is a copy of the "binary counter" demo compiled and linked to
run at flash address 0x8000 instead of 0x0000. This is useful for users who
would like to run their applications with a permanent bootloader program in
the first page of flash.

Please see the linker configuration files for an example of how this offset
can be built into the compilation process.

SWO is configured in 1M baud, 8-n-1 mode.


******************************************************************************


