//*****************************************************************************
//
//! @file adc_lpmode2.c
//!
//! @brief Example that takes samples with the ADC at 1Hz and powers off the
//! ADC between samples.  CTIMER-A1 is used to drive the process.
//!
//! SWO is configured in 1M baud, 8-n-1 mode.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define ADC_EXAMPLE_DEBUG 1

//*****************************************************************************
//
// Start up the ITM interface.
//
//*****************************************************************************
void
itm_start(void)
{
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();
}

//*****************************************************************************
//
// Set up the core for sleeping, and then go to sleep.
//
//*****************************************************************************
void
sleep(void)
{
  //
  // Disable things that can't run in sleep mode.
  //
#if (0 == ADC_EXAMPLE_DEBUG)
  am_bsp_debug_printf_disable();
#endif

  //
  // Go to Deep Sleep.
  //
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

  //
  // Re-enable peripherals for run mode.
  //
#if (0 == ADC_EXAMPLE_DEBUG)
  am_bsp_debug_printf_enable();
#endif
}

//*****************************************************************************
//
// Configure the ADC.
//
//*****************************************************************************
void
adc_config(void)
{
    am_hal_adc_config_t sADCConfig;

    //
    // Enable the ADC power domain.
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_ADC);

    //
    // Set up the ADC configuration parameters. These settings are reasonable
    // for accurate measurements at a low sample rate.
    //
    sADCConfig.ui32Clock = AM_HAL_ADC_CLOCK_HFRC;
    sADCConfig.ui32TriggerConfig = AM_HAL_ADC_TRIGGER_SOFT;
    sADCConfig.ui32Reference = AM_HAL_ADC_REF_INT_2P0;
    sADCConfig.ui32ClockMode = AM_HAL_ADC_CK_LOW_POWER;
    sADCConfig.ui32PowerMode = AM_HAL_ADC_LPMODE_0;
    sADCConfig.ui32Repeat = AM_HAL_ADC_NO_REPEAT;

    am_hal_adc_config(&sADCConfig);

    //
    // For this example, the samples will be coming in slowly. This means we
    // can afford to wake up for every conversion.
    //
    am_hal_adc_int_enable(AM_HAL_ADC_INT_CNVCMP);

    //
    // Set up an ADC slot
    //
    am_hal_adc_slot_config(0, AM_HAL_ADC_SLOT_AVG_1 |
                              AM_HAL_ADC_SLOT_14BIT |
                              AM_HAL_ADC_SLOT_CHSEL_SE0 |
                              AM_HAL_ADC_SLOT_ENABLE);
    //
    // Enable the ADC.
    //
    am_hal_adc_enable();
}

//*****************************************************************************
//
// Start sampling
//
//*****************************************************************************
void
init_timerA1_for_ADC(void)
{
    //
    // Start a timer to trigger the ADC periodically. This timer won't actually
    // be connected to the ADC (as can be done with Timer 3). Instead, we'll
    // generate interrupts to the CPU, and then use the CPU to trigger the ADC
    // in the CTIMER interrupt handler.
    //
    am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
                                   AM_HAL_CTIMER_LFRC_512HZ |
                                   AM_HAL_CTIMER_FN_REPEAT |
                                   AM_HAL_CTIMER_INT_ENABLE);

    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 511, 0);


    //
    // Start the timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Interrupt handler for the CTIMER
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear TimerA0 Interrupt.
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Re-configure the ADC. We lose configuation data in the power-down, so
    // we'll reconfigure the ADC here. If you don't shut down the ADC, this
    // step is unnecessary.
    //
    adc_config();

    //
    // Trigger the ADC
    //
    am_hal_adc_trigger();
}

//*****************************************************************************
//
// Interrupt handler for the ADC.
//
//*****************************************************************************
void
am_adc_isr(void)
{
    uint32_t ui32Status, ui32FifoData;

    //
    // Read the interrupt status.
    //
    ui32Status = am_hal_adc_int_status_get(true);

    //
    // Clear the ADC interrupt.
    //
    am_hal_adc_int_clear(ui32Status);

    //
    // If we got a conversion completion interrupt (which should be our only
    // ADC interrupt), go ahead and read the data.
    //
    if (ui32Status & AM_HAL_ADC_INT_CNVCMP)
    {
      //
      // Read the value from the FIFO into the circular buffer.
      //
      ui32FifoData = am_hal_adc_fifo_pop();
#if (1 == ADC_EXAMPLE_DEBUG)
      am_util_stdio_printf("ADC Slot =  %d\n", AM_HAL_ADC_FIFO_SLOT(ui32FifoData));
      am_util_stdio_printf("ADC Value = %8.8X\n", AM_HAL_ADC_FIFO_SAMPLE(ui32FifoData));
#endif  
    }
    
    //
    // Power down the ADC module
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_ADC);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the system clock to maximum frequency, and set the default low-power
    // settings for this board.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);
    am_hal_pwrctrl_bucks_enable();
    am_hal_vcomp_disable();

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Start the ITM interface.
    //
    itm_start();

    //
    // Set a pin to act as our ADC input
    //
    am_hal_gpio_pin_config(16, AM_HAL_PIN_16_ADCSE0);

    //
    // Start the timer-based ADC measurements.
    //
    init_timerA1_for_ADC();

    //
    // Enable interrupts.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_ADC);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);

    am_hal_interrupt_master_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("ADC Example at 1Hz with ADC disabled between samples\n");

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
#if (0 == ADC_EXAMPLE_DEBUG)
    am_bsp_debug_printf_disable();
#endif
    
    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Disable interrupts
        //
        am_hal_interrupt_master_disable();

        //
        // Put the core to sleep.
        //
        sleep();

        //
        // Enable interrupts.
        //
        am_hal_interrupt_master_enable();
    }
}
