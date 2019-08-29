@Echo off

Echo.
Echo If a problem is encountered with running the binary, check the .bin file
Echo for the first two vector entries and update the
Echo commander batch file appropriately.
Echo.
:: Echo Press any key to continue.
:: pause

For %%D in (k, K, keil, Keil, KEIL) Do If "%1" == "%%D" Goto Keil
For %%D in (i, I, iar,  Iar,  IAR ) Do If "%1" == "%%D" Goto IAR
For %%D in (g, G, gcc,  Gcc,  GCC ) Do If "%1" == "%%D" Goto GCC
:: Else default to Keil

:Keil
Echo Running KEIL binary
Set JLnk=selftest_commander_keil.jlink
goto runscript

:IAR
Echo Running IAR binary
Set JLnk=selftest_commander_iar.jlink
goto runscript

:GCC
Echo Running GCC binary
Set JLnk=selftest_commander_gcc.jlink
goto runscript

:runscript
Echo.
jlink -CommanderScript %JLnk%
goto End

:End
Echo.
Echo The result should be FAExxxxx, where xxxxx is 00000 for PASS.
Echo The next 2 values represent the run time as ss.fff, where ss is the
Echo whole number of seconds and fff is the fractional part to 3 decimals.
Echo.

Set JLnk=

:: pause
