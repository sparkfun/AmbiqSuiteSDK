//*****************************************************************************
//
//  am_reg_macros_asm.h
//! @file
//!
//! @brief Inline assembly macros. Initially for critical section handling in
//! protecting hardware registers.
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

#ifndef AM_REG_MACROS_ASM_H
#define AM_REG_MACROS_ASM_H

#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//
// Critical section assembly macros
//
// These macros implement critical section protection using inline assembly
// for various compilers.  They are intended to be used in other register
// macros or directly in sections of code.
//
// Important usage note: These macros create a local scope and therefore MUST
// be used in pairs.
//
//*****************************************************************************
#define AM_CRITICAL_BEGIN                                                   \
    if ( 1 )                                                                \
    {                                                                       \
        volatile uint32_t ui32Primask_04172010;                             \
        ui32Primask_04172010 = am_hal_interrupt_master_disable();

#define AM_CRITICAL_END                                                     \
        am_hal_interrupt_master_set(ui32Primask_04172010);                  \
    }

//*****************************************************************************
//
// A collection of some common inline assembly instructions / intrinsics.
//
//*****************************************************************************
//
// AM_ASM_BKPT(n)
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_BKPT(n)  __breakpoint(n)
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_BKPT(n)  __asm("    bkpt "#n);
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_BKPT(n)  __asm("    bkpt "#n);
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_BKPT(n)  asm("    bkpt "#n);
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_WFI
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_WFI      __wfi();
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_WFI      __asm("    wfi");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_WFI      __asm("    wfi");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_WFI      asm("    wfi");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_WFE
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_WFE      __wfe();
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_WFE      __asm("    wfe");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_WFE      __asm("    wfe");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_WFE      asm("    wfe");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_SEV
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_SEV      __sev();
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_SEV      __asm("    sev");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_SEV      __asm("    sev");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_SEV      asm("    sev");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_NOP
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_NOP      __nop();
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_NOP      __asm("    nop");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_NOP      __asm("    nop");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_NOP      asm("    nop");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_DSB
// In cmsis_armcc.h, __DSB() is defined as __dsb(0xF).
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_DSB      __dsb(15);
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_DSB      __asm("    dsb #15");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_DSB      __asm("    dsb #15");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_DSB      asm("    dsb #15");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//
// AM_ASM_ISB
// In cmsis_armcc.h, __ISB() is defined as __isb(0xF).
//
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
#define     AM_ASM_ISB      __isb(15);
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#define     AM_ASM_ISB      __asm("    isb #15");
#elif defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_ISB      __asm("    isb #15");
#elif   defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_ISB      asm("    isb #15");
#else
#error Compiler is unknown, please contact Ambiq support team
#endif


#ifdef __cplusplus
}
#endif

#endif // AM_REG_MACROS_ASM_H

