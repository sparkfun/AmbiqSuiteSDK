//*****************************************************************************
//
//  am_hal_sysctrl.c
//! @file
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup sysctrl1 System Control (SYSCTRL)
//! @ingroup apollo1hal
//! @{
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @brief Place the core into sleep or deepsleep.
//!
//! @param bSleepDeep - False for Normal or True Deep sleep.
//!
//! This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//!
//! Valid values for bSleepDeep are:
//!
//!     AM_HAL_SYSCTRL_SLEEP_NORMAL
//!     AM_HAL_SYSCTRL_SLEEP_DEEP
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_sleep(bool bSleepDeep)
{
    //
    // If the user selected DEEPSLEEP and the TPIU is off, attempt to enter
    // DEEP SLEEP.
    //
    if ((bSleepDeep == AM_HAL_SYSCTRL_SLEEP_DEEP) &&
        (AM_BFM(MCUCTRL, TPIUCTRL, ENABLE) == AM_REG_MCUCTRL_TPIUCTRL_ENABLE_DIS))
    {
        //
        // Prepare the core for deepsleep (write 1 to the DEEPSLEEP bit).
        //
        AM_BFW(SYSCTRL, SCR, SLEEPDEEP, 1);
    }
    else
    {
        AM_BFW(SYSCTRL, SCR, SLEEPDEEP, 0);
    }

    //
    // Before executing WFI, flush any buffered core and peripheral writes.
    //
    AM_ASM_DSB

    //
    // Go to sleep.
    //
    AM_ASM_WFI;

    //
    // Upon wake, execute the Instruction Sync Barrier instruction.
    //
    AM_ASM_ISB
}

//*****************************************************************************
//
//! @brief Place the core into the deepest sleep state possible
//!
//! This function puts the MCU to sleep or deepsleep depending on which
//! peripherals are on. If the UART or either IOM module is enabled, the MCU
//! will be placed into normal sleep. Otherwise, the MCU will go to deep sleep.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_sleep_auto(void)
{
    //
    // If any of the HFRC peripherals are on, go to normal sleep. Otherwise go
    // to deep sleep.
    //
    if (AM_BFM(MCUCTRL, TPIUCTRL, ENABLE) || AM_BFMn(IOMSTR, 0, CFG, IFCEN) ||
        AM_BFMn(IOMSTR, 1, CFG, IFCEN) || AM_REG(CLKGEN, UARTEN))
    {
        AM_BFW(SYSCTRL, SCR, SLEEPDEEP, 1);
    }
    else
    {
        AM_BFW(SYSCTRL, SCR, SLEEPDEEP, 0);
    }

    //
    // Before executing WFI, flush any buffered core and peripheral writes.
    //
    AM_ASM_DSB

    //
    // Go to sleep.
    //
    AM_ASM_WFI;

    //
    // Upon wake, execute the Instruction Sync Barrier instruction.
    //
    AM_ASM_ISB
}
//*****************************************************************************
//
//! @brief Enable the floating point module.
//!
//! Call this function to enable the ARM hardware floating point module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_enable(void)
{
    //
    // Enable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    AM_REG(SYSCTRL, CPACR) = (AM_REG_SYSCTRL_CPACR_CP11(0x3) |
                              AM_REG_SYSCTRL_CPACR_CP10(0x3));
}

//*****************************************************************************
//
//! @brief Disable the floating point module.
//!
//! Call this function to disable the ARM hardware floating point module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_disable(void)
{
    //
    // Disable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    AM_REG(SYSCTRL, CPACR) = 0x00000000                     &
                             ~(AM_REG_SYSCTRL_CPACR_CP11(0x3) |
                               AM_REG_SYSCTRL_CPACR_CP10(0x3));
}

//*****************************************************************************
//
//! @brief Enable stacking of FPU registers on exception entry.
//!
//! @param bLazy - Set to "true" to enable "lazy stacking".
//!
//! This function allows the core to save floating-point information to the
//! stack on exception entry. Setting the bLazy option enables "lazy stacking"
//! for interrupt handlers.  Normally, mixing floating-point code and interrupt
//! driven routines causes increased interrupt latency, because the core must
//! save extra information to the stack upon exception entry. With the lazy
//! stacking option enabled, the core will skip the saving of floating-point
//! registers when possible, reducing average interrupt latency.
//!
//! @note At reset of the Cortex M4, the ASPEN and LSPEN bits are set to 1,
//! enabling Lazy mode by default. Therefore this function will generally
//! only have an affect when setting for full-context save (or when switching
//! from full-context to lazy mode).
//!
//! @note See also:
//! infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0298a/DAFGGBJD.html
//!
//! @note Three valid FPU context saving modes are possible.
//! 1. Lazy           ASPEN=1 LSPEN=1 am_hal_sysctrl_fpu_stacking_enable(true)
//!                                   and default.
//! 2. Full-context   ASPEN=1 LSPEN=0 am_hal_sysctrl_fpu_stacking_enable(false)
//! 3. No FPU state   ASPEN=0 LSPEN=0 am_hal_sysctrl_fpu_stacking_disable()
//! 4. Invalid        ASPEN=0 LSPEN=1
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_enable(bool bLazy)
{
#define SYSCTRL_FPCCR_LAZY  (AM_REG_SYSCTRL_FPCCR_ASPEN_M | AM_REG_SYSCTRL_FPCCR_LSPEN_M)

    uint32_t ui32fpccr;

    //
    // Set the requested FPU stacking mode in ISRs.
    //
    AM_CRITICAL_BEGIN
    ui32fpccr  = AM_REG(SYSCTRL, FPCCR);
    ui32fpccr &= ~(SYSCTRL_FPCCR_LAZY);
    ui32fpccr |= (bLazy ? SYSCTRL_FPCCR_LAZY : AM_REG_SYSCTRL_FPCCR_ASPEN_M);
    AM_REG(SYSCTRL, FPCCR) = ui32fpccr;
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief Disable FPU register stacking on exception entry.
//!
//! This function disables all stacking of floating point registers for
//! interrupt handlers.  This mode should only be used when it is absolutely
//! known that no FPU instructions will be executed in an ISR.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_disable(void)
{
    //
    // Completely disable FPU context save on entry to ISRs.
    //
    AM_CRITICAL_BEGIN
    AM_REG(SYSCTRL, FPCCR) &= ~SYSCTRL_FPCCR_LAZY;
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//!
//! This function issues a system wide reset (Apollo POR level reset).
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_reset(void)
{
    //
    // Set the system reset bit in the AIRCR register
    //
    AM_REG(SYSCTRL, AIRCR) = AM_REG_SYSCTRL_AIRCR_VECTKEY(0x5FA) |
                             AM_REG_SYSCTRL_AIRCR_SYSRESETREQ(1);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
