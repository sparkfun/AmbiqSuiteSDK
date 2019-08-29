Name:
=====
 uart_printf


Description:
============
 Example that uses the UART interface for printf.


This example walks through the ASCII table (starting at character 033('!')
and ending on 126('~')) and prints the character to the UART. This output
can be decoded by running AM Flash and configuring the console for UART at
115200 Baud. This example works by configuring a timer and printing a new
character after ever interrupt and sleeps in between timer interrupts.


******************************************************************************


