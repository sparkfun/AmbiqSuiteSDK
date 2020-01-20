Name:
=====
 ble_freertos_amota


Description:
============
 Example of the freertos amota app running under FreeRTOS.


This example implements a BLE heart rate sensor within the FreeRTOS
framework. To save power, this application is compiled without print
statements by default. To enable them, add the following project-level
macro definitions.

AM_DEBUG_PRINTF
WSF_TRACE_ENABLED=1

If enabled, debug messages will be sent over ITM.


******************************************************************************


