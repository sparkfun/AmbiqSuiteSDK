Name:
=====
 buckzx_demo


Description:
============
 Demonstrate operation of the buck zero-cross implementation.


See Errata ERR019 for additional details on this issue.

Heavily based on deepsleep_wake, this example demonstrates the
operation of the buck zero-cross implementation.

The example uses 5 GPIOs in total for the demonstration as follows:
GPIO 3: Shows the CTIMER A buck pulse.
GPIO 4: Shows The CTIMER B buck pulse.
GPIO 5: Demarcates the sleep cycle; high while sleeping,
low when awake.
GPIO 6: Demarcates am_ctimer_isr(), high when the ISR is
entered, low on exit. Basically envelopes each of
the buck pulses.
GPIO 7: Toggling pattern of approximately 1us width during
the time the bucks are being restored.

To run the buckzx_demo example:
- Connect an analyzer to GPIOs 3-7 on the apollo2_evb board with
a trigger (high-to-low) on GPIO5.
- Flash the example binary and reset.
- As on deepsleep_wake, the Apollo2 will incur an RTC interrupt
once every second. It will stay awake for one second, then it
will deep sleep for one second.
- The sleep time is indicated by GPIO 5 being high. The awake
time is indicated by GPIO 5 being low.
- Zooming in on the GPIO5 low-going pulse shows the buck handling
in action.
- GPIO 3 & 4 will each pulse once during the wake time. These
pulses show the core and memory bucks.
Note that the 2 signals occur differently depending on the
CTIMER used.  CTIMERs 0 and 1 will see core buck on GPIO4
(CTimer B) and mem buck on GPIO3 (CTimer A).  CTIMERs 2 and 3
are vice-versa.
- GPIO 6 pulses each time the CTIMER fires.  Therefore, there
should be as many GPIO6 pulses as 3 & 4 combined.
- GPIO 7 simply demonstrates MCU usage while the bucks are resumed.
- Modify BUCK_TIMER (0 - 3) to change the timer that is actually
used for the operation.


******************************************************************************


