Name:
=====
 burst_mode


Description:
============
 Example demonstrates the usage of Burst mode (a.k.a. TurboSpot) HAL.


Purpose:
========
This example shows how to detect if Burst Mode is available.  If so,
it sets the Apollo3 into Normal (48MHz) mode, then times a calculation of 
prime numbers, displaying the elapsed time.  Next, it switches the Apollo3
into Burst mode, performs the same calculation, then displays the elapsed
time which should be roughly 50% of Normal mode.

Printing takes place over the ITM at 1M Baud.



******************************************************************************


