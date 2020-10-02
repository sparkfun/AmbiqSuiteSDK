Name:
=====
 mspi_power_example


Description:
============
 MSPI Power Profiling Example


Purpose:
========
This example is designed to provide a convenient way to assess the
expected power to be used by the MSPI device during: a) DMA reads, b) DMA writes
and c) XIP operation.  The example is designed to be combined with and load the
prime number library to act as the executable in external Quad SPI flash.

Additional Information:
=======================

1. Build the libprime.a with the project in \libs\prime\iar.

2. Generate prime_mpi.bin using \libs\prime\make_lib.bat

3. Copy prime_mpi.bin into \boards\common3\examples\mspi_power_example

4. Build the mspi_power_example binaries

5. Create the binary with mspi_flash_loader + external executable from Step #1.

./mspi_testcase_binary_combiner.py --loaderbin iar/bin/mspi_power_example.bin --libbin prime_mpi.bin --off 0x0 --p0 1000 --p1 0 --p2 1 --r 168 --outbin mspi_power_executable --loader-address 0x0000C000 --chipType apollo3p

This example can work on:
Apollo3p_evb + Cygnus
Recommend to use 3.3V power supply voltage.



******************************************************************************


