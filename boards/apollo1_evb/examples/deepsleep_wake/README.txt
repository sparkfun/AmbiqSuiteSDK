Name:
=====
 deepsleep_wake


Description:
============
 Example that goes to deepsleep and wakes from either the RTC or GPIO.


This example configures the device to go into a deep sleep mode. Once in
deep sleep the device has the ability to wake from button 0 or
the RTC configured to interrupt every second. If the MCU woke from a button
press, it will toggle LED0. If the MCU woke from the RTC, it will toggle
LED1.

The example begins by printing out a banner annoucement message through
the UART, which is then completely disabled for the remainder of execution.

Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
Please note that text end-of-line is a newline (LF) character only.
Therefore, the UART terminal must be set to simulate a CR/LF.


******************************************************************************


