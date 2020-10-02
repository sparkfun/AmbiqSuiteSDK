//*****************************************************************************
//
//! @file adc_vbatt.c
//!
//! @brief ADC VBATT voltage divider, BATT load, and temperature reading example.
//!
//! This example initializes the ADC, and a timer. Two times per second it
//! reads the VBATT voltage divider and temperature sensor and prints them.
//! It monitors button 0 and if pressed, it turns on the BATT LOAD resistor.
//! One should monitor MCU current to see when the load is on or off.
//!
//! Printing takes place over the ITM at 1M Baud.
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

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
// Sample Count semaphore from ADC ISR to base level.
uint32_t g_ui32SampleCount;

// ADC code for voltage divider from ADC ISR to base level.
uint16_t g_ui16ADCVDD_code;

// ADC code for temperature sensor from ADC ISR to base level.
uint16_t g_ui16ADCTEMP_code;

//*****************************************************************************
//
// ADC Configuration
//
//*****************************************************************************
am_hal_adc_config_t g_sADC_CfgA =
{
#if AM_PART_APOLLO2
    //
    // Select the ADC Clock source.
    //
    .ui32Clock = AM_HAL_ADC_CLOCK_DIV2,

    //
    // Select the ADC trigger source using a trigger source macro.
    //
    .ui32TriggerConfig = AM_HAL_ADC_TRIGGER_SOFT,

    //
    // Select the ADC reference voltage.
    //
    .ui32Reference = AM_HAL_ADC_REF_INT_1P5,
    .ui32ClockMode = AM_HAL_ADC_CK_LOW_POWER,

    //
    // Choose the power mode for the ADC's idle state.
    //
    .ui32PowerMode = AM_HAL_ADC_LPMODE_1,

    //
    // Enable repeating samples using Timer3A.
    //
    .ui32Repeat = AM_HAL_ADC_REPEAT
#else
    // Select the ADC Clock source using one of the clock source macros.
    AM_HAL_ADC_CLOCK_1_5MHZ,

    // Select the ADC trigger source using a trigger source macro.
    AM_HAL_ADC_TRIGGER_SOFT,

    // Use a macro to select the ADC reference voltage.
    AM_HAL_ADC_REF_INT,

    // Use a macro to choose a maximum sample rate setting.
    AM_HAL_ADC_MODE_1MSPS,

    // Use a macro to choose the power mode for the ADC's idle state.
    AM_HAL_ADC_LPMODE_2,

    // Use the Repeat macro to enable repeating samples using Timer3A.
    AM_HAL_ADC_REPEAT,

    // Power Off the temp sensor.
    AM_HAL_ADC_PON_TEMP,

    // Set the ADC window limits using the window limit macro.
    AM_HAL_ADC_WINDOW(768, 256) // arbitrary window setting, not used here.
#endif
};

//*****************************************************************************
//
// Timer configurations.
//
//*****************************************************************************
am_hal_ctimer_config_t g_sTimer3 =
{
    // do not link A and B together to make a long 32-bit counter.
    0,

    // Set up timer 3A to drive the ADC
    (AM_HAL_CTIMER_FN_PWM_REPEAT |
     AM_HAL_CTIMER_LFRC_32HZ),

    // Timer 3B is not used in this example.
    0,
};

//*****************************************************************************
//
// ADC Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_adc_isr(void)
{
#if AM_PART_APOLLO2
    uint32_t ui32fifodata;
    volatile uint32_t ui32Status;

    //
    // Clear timer 3 interrupt.
    //
    ui32Status = am_hal_adc_int_status_get(true);
    am_hal_adc_int_clear(ui32Status);

    //
    // Toggle LED 3.
    //
    am_devices_led_toggle(am_bsp_psLEDs, 3);

    //
    // Keep grabbing value from the ADC FIFO until it goes empty.
    //
    while ( AM_HAL_ADC_FIFO_COUNT(am_hal_adc_fifo_peek()) )
    {
        ui32fifodata = am_hal_adc_fifo_pop();

        //
        // Select which one of the two enabled slots is here right now.
        //
        if ( AM_HAL_ADC_FIFO_SLOT(ui32fifodata) == 5)
        {
            //
            // Just grab the ADC code for the battery voltage divider.
            //
            g_ui16ADCVDD_code = AM_HAL_ADC_FIFO_FULL_SAMPLE(ui32fifodata);
        }
        else
        {
            //
            // Just grab the ADC code for the temperature sensor.
            // We need the integer part in the low 16-bits.
            //
            g_ui16ADCTEMP_code = AM_HAL_ADC_FIFO_FULL_SAMPLE(ui32fifodata) & 0xFFC0;
        }
    }
#else
    am_hal_adc_fifo_read_t fifo_info;

    //
    // Clear ADC Interrupt (write to clear).
    //
    AM_REGn(CTIMER, 0, INTCLR) = AM_REG_CTIMER_INTCLR_CTMRA0INT_M;

    //
    // Toggle LED 3.
    //
    am_devices_led_toggle(am_bsp_psLEDs, 3);

    //
    // Keep grabbing value from the ADC FIFO until it goes empty.
    //
    while (am_hal_adc_fifo_read(&fifo_info))
    {
        //
        // Select which one of the two enabled slots is here right now.
        //
        if (fifo_info.ui8Slot == 5)
        {
            //
            // Just grab the ADC code for the battery voltage divider.
            //
            g_ui16ADCVDD_code = fifo_info.ui16Data;
        }
        else
        {
            //
            // Just grab the ADC code for the temperature sensor.
            //
            g_ui16ADCTEMP_code = fifo_info.ui16Data;
        }
    }
#endif // AM_PART_APOLLO2

    //
    // Signal interrupt arrival to base level.
    //
    g_ui32SampleCount++;
}

//*****************************************************************************
//
// ADC INIT Function
//
//*****************************************************************************
void
adc_init(void)
{
#if AM_PART_APOLLO
    //
    // We MUST turn on band gap to use the Temp Sensor.
    // The ADC hardware in mode 2 will cycle the power to the bandgap
    // automatically.
    //
    am_hal_mcuctrl_bandgap_enable();
#endif

#if AM_PART_APOLLO2
    //
    // Enable the ADC power domain.
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_ADC);
#endif

    //
    // Configure the ADC.
    //
    am_hal_adc_config(&g_sADC_CfgA);

    //
    // Initialize the slot control registers.
    //
    am_hal_adc_slot_config(0, 0); // unused slot
    am_hal_adc_slot_config(1, 0); // unused slot
    am_hal_adc_slot_config(2, 0); // unused slot
    am_hal_adc_slot_config(3, 0); // unused slot
    am_hal_adc_slot_config(4, 0); // unused slot
    am_hal_adc_slot_config(5, (AM_HAL_ADC_SLOT_AVG_1        |
                               AM_HAL_ADC_SLOT_CHSEL_VBATT  |
                               AM_HAL_ADC_SLOT_WINDOW_EN    |
                               AM_HAL_ADC_SLOT_ENABLE));

    am_hal_adc_slot_config(6, 0); // unused slot
#if AM_PART_APOLLO
    am_hal_adc_slot_config(7, (AM_HAL_ADC_SLOT_AVG_1            |
                               AM_HAL_ADC_SLOT_CHSEL_TEMP       |
                               AM_HAL_ADC_SLOT_WINDOW_EN        |
                               AM_HAL_ADC_SLOT_ENABLE));
#endif
#if AM_PART_APOLLO2
    am_hal_adc_slot_config(7, (AM_HAL_ADC_SLOT_AVG_1            |
                               AM_HAL_ADC_SLOT_CHSEL_TEMP       |
                               AM_HAL_ADC_SLOT_10BIT            |
                               AM_HAL_ADC_SLOT_WINDOW_EN        |
                               AM_HAL_ADC_SLOT_ENABLE));
#endif

    //
    // Enable the ADC.
    //
    am_hal_adc_enable();
}

//*****************************************************************************
//
// Enable the ADC INIT TIMER 3A function and set for 0.5 second period.
//
//*****************************************************************************
void
adc_init_timer3A(void)
{
    uint32_t ui32Period = 2000; // Set for 2 second (2000ms) period

    //
    // LFRC has to be turned on for this example because we are running this
    // timer off of the LFRC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Set up timer 3A so start by clearing it.
    //
    am_hal_ctimer_clear(3, AM_HAL_CTIMER_TIMERA);

    //
    // Configure the timer to count 32Hz LFRC clocks but don't start it yet.
    //
    am_hal_ctimer_config(3, &g_sTimer3);

    //
    // Compute CMPR value needed for desired period based on a 32HZ clock.
    //
    ui32Period = ui32Period * 32 / 1000;
    am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA,
                             ui32Period, (ui32Period >> 1));

    //
    // Enable the timer output "pin". This refers to the pin as seen from
    // inside the timer. The actual GPIO pin is neither enabled nor driven.
    //
    am_hal_ctimer_pin_enable(3, AM_HAL_CTIMER_TIMERA);

    //
    // Set up timer 3A as the trigger source for the ADC.
    //
    am_hal_ctimer_adc_trigger_enable();

    //
    // Start timer 3A.
    //
    am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    bool  bMeasured;
    float fTempF;
    int32_t i32BaseLevelCount;
    const float fReferenceVoltage = 1.5;
    float fVBATT;
    float fADCTempVolts;
    float fADCTempDegreesC;
    float fTemp, fVoltage, fOffset;

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize device drivers for the LEDs on the board.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);

    //
    // Configure the Button 0 pin as a simple GPIO input.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_BUTTON0, AM_HAL_PIN_INPUT);

    //
    // Initialize the SWO GPIO pin.
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("ADC VBATT and Temperature Sensing Example.\n");

    //
    // Enable floating point.
    //
    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);

    //
    // Initialize CTIMER 3A to trigger the ADC every 0.5 seconds.
    //
    adc_init_timer3A();

    //
    // Print out ctimer initial register state.
    //
    am_util_stdio_printf("\n");
    am_util_stdio_printf("CTIMER3=0x%08X @ 0x%08X\n",
                         AM_REGVAL(REG_CTIMER_BASEADDR + AM_REG_CTIMER_TMR3_O),
                         REG_CTIMER_BASEADDR + AM_REG_CTIMER_TMR3_O);
    am_util_stdio_printf("CTIMER3=0x%08X @ 0x%08X\n",
                         AM_REGVAL(REG_CTIMER_BASEADDR + AM_REG_CTIMER_CMPRA3_O),
                         REG_CTIMER_BASEADDR + AM_REG_CTIMER_CMPRA3_O);
    am_util_stdio_printf("CTIMER3=0x%08X @ 0x%08X\n",
                         AM_REGVAL(REG_CTIMER_BASEADDR + AM_REG_CTIMER_CMPRB3_O),
                         REG_CTIMER_BASEADDR + AM_REG_CTIMER_CMPRB3_O);
    am_util_stdio_printf("CTIMER3=0x%08X @ 0x%08X\n",
                         AM_REGVAL(REG_CTIMER_BASEADDR + AM_REG_CTIMER_CTRL3_O),
                         REG_CTIMER_BASEADDR + AM_REG_CTIMER_CTRL3_O);

    //
    // Initialize the ADC.
    //
    adc_init();

    //
    // Print out ADC initial register state.
    //
    am_util_stdio_printf("\n");
    am_util_stdio_printf("ADC REGISTERS @ 0x%08X\n", (uint32_t)REG_ADC_BASEADDR);
    am_util_stdio_printf("ADC CFG   = 0x%08X\n", AM_REG(ADC, CFG));
    am_util_stdio_printf("ADC SLOT0 = 0x%08X\n", AM_REG(ADC, SL0CFG));
    am_util_stdio_printf("ADC SLOT1 = 0x%08X\n", AM_REG(ADC, SL1CFG));
    am_util_stdio_printf("ADC SLOT2 = 0x%08X\n", AM_REG(ADC, SL2CFG));
    am_util_stdio_printf("ADC SLOT3 = 0x%08X\n", AM_REG(ADC, SL3CFG));
    am_util_stdio_printf("ADC SLOT4 = 0x%08X\n", AM_REG(ADC, SL4CFG));
    am_util_stdio_printf("ADC SLOT5 = 0x%08X\n", AM_REG(ADC, SL5CFG));
    am_util_stdio_printf("ADC SLOT6 = 0x%08X\n", AM_REG(ADC, SL6CFG));
    am_util_stdio_printf("ADC SLOT7 = 0x%08X\n", AM_REG(ADC, SL7CFG));
#if AM_PART_APOLLO
    am_util_stdio_printf("ADC WLIM  = 0x%08X\n", AM_REG(ADC, WLIM));
#endif

    //
    // Print out the temperature trim values as recorded in OTP.
    //
    bMeasured = am_hal_adc_temp_trims_get(&fTemp, &fVoltage, &fOffset);
    am_util_stdio_printf("\n");
    am_util_stdio_printf("TRIMMED TEMP    = %.3f\n", fTemp);
    am_util_stdio_printf("TRIMMED VOLTAGE = %.3f\n", fVoltage);
    am_util_stdio_printf("TRIMMED Offset  = %.3f\n", fOffset);
    am_util_stdio_printf("Note - these trim values are '%s' values.\n",
                         bMeasured ? "calibrated" : "uncalibrated default");
    am_util_stdio_printf("\n");

    //
    // Enable the ADC interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_ADC);
    am_hal_interrupt_master_enable();

    //
    // Enable the ADC interrupts in the ADC.
    //
    am_hal_adc_int_enable(AM_REG_ADC_INTEN_WCINC(1)     |
                          AM_REG_ADC_INTEN_WCEXC(1)     |
                          AM_REG_ADC_INTEN_FIFOOVR2(1)  |
                          AM_REG_ADC_INTEN_FIFOOVR1(1)  |
                          AM_REG_ADC_INTEN_SCNCMP(1)    |
                          AM_REG_ADC_INTEN_CNVCMP(1));

    //
    // Reset the sample count which will be incremented by the ISR.
    //
    g_ui32SampleCount = 0;

    //
    // Kick Start Timer 3 with an ADC software trigger in REPEAT used.
    //
    am_hal_adc_trigger();

    //
    // Track buffer depth for progress messages.
    //
    i32BaseLevelCount = g_ui32SampleCount;

    //
    // Wait here for the ISR to grab a buffer of samples.
    //
    while (1)
    {
        //
        // Print the battery voltage and temperature for each interrupt
        //
        if (g_ui32SampleCount > i32BaseLevelCount)
        {
            i32BaseLevelCount = g_ui32SampleCount;

            //
            // Compute the voltage divider output.
            //
            fVBATT = ((float)g_ui16ADCVDD_code) * 3.0f * fReferenceVoltage
                    / 1024.0f / 64.0f;

            //
            // Print the voltage divider output.
            //
            am_util_stdio_printf("VBATT = <%.3f> (0x%04X) ",
                                 fVBATT, g_ui16ADCVDD_code);

            //
            // Convert and scale the temperature.
            // Temperatures are in Fahrenheit range -40 to 225 degrees.
            // Voltage range is 0.825V to 1.283V
            // First get the ADC voltage corresponding to temperature.
            //
            fADCTempVolts = ((float)g_ui16ADCTEMP_code) * fReferenceVoltage
                          / (1024.0f * 64.0f);

            //
            // Now call the HAL routine to convert volts to degrees Celsius.
            //
            fADCTempDegreesC = am_hal_adc_volts_to_celsius(fADCTempVolts);

            //
            // print the temperature value in Celsius.
            //
            am_util_stdio_printf("TEMP = %.2f C (0x%04X) ",
                                 fADCTempDegreesC, g_ui16ADCTEMP_code);

            //
            // Print the temperature value in Fahrenheit.
            //
            fTempF = (fADCTempDegreesC * (9.0f / 5.0f)) + 32.0f;
            am_util_stdio_printf(" %.2f F", fTempF);

            //
            // Use button 0 to turn on or off the battery load resistor.
            //
#if AM_PART_APOLLO
            if (!am_hal_gpio_input_bit_read(AM_BSP_GPIO_BUTTON0))
            {
                am_util_stdio_printf("BATTERY LOAD RESISTOR ON\n");
                am_hal_adc_batt_load_enable();
                am_devices_led_on(am_bsp_psLEDs, 2);
            }
            else
            {
                am_util_stdio_printf("\n");
                am_hal_adc_batt_load_disable();
                am_devices_led_off(am_bsp_psLEDs, 2);
            }
#else
            am_util_stdio_printf("\n");
#endif
        }

        //
        // Sleep here until the next ADC interrupt comes along.
        //
        am_devices_led_off(am_bsp_psLEDs, 0);
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_devices_led_on(am_bsp_psLEDs, 0);
    }
}
