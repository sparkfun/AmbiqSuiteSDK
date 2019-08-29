Name:
=====
 em9304_test_bridge


Description:
============
 UART-to-SPI bridge for Bluetooth Direct Mode testing of EM9304.


This project implements a UART to SPI bridge for Direct Mode testing of
the EM9304 BLE Controller.  The project uses UART0 and IOM5 in SPI mode.
HCI packets are provided over the UART which the Apollo2 transfers via
the SPI interface according to the EM9304 data sheet.  Responses from the
EM9304 are read from the SPI interface and relayed over the UART.  The
project uses the FIFOs and interrupts of the UART and IOM in order to
implement non-blocking processing of the next received packet from either
interface.


******************************************************************************


