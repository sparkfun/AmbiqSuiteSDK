//*****************************************************************************
//
// Copyright (c) 2017, Ambiq Micro
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
// This is part of revision v1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "usr_include.h"
#include "am_app_KWD_init.h"
//#include "am_app_utils_task.h"
#if USE_OUTPUT_AMVOS_AMA
#include "am_app_KWD_AMA.h"
#endif

#if configUSE_SENSORY_THF_ONLY || configUSE_Sensory_THF
//#include "SensoryLib.h"
#endif
//*****************************************************************************
//
// KWD application tasks list. User needs to keep this lists name unchaged.
//
//*****************************************************************************

#if USE_OUTPUT_AMVOS_AMA
DeviceInformation g_amaDeviceInformation = 
{
    .serial_number  = "11223355",
    .name           = "VoS-AMA-LE",
    .device_type    = "A2Y04QPFCANLPQ"
};

#if configUSE_AUDIO_PRE_BUFF_AMA
bool g_config_preroll_enabled = true;
#else
bool g_config_preroll_enabled = false;
#endif // configUSE_AUDIO_PRE_BUFF_AMA
#endif // #if USE_OUTPUT_AMVOS_AMA

bool g_config_opus_enabled = false;


//*****************************************************************************
//
// KWD application tasks list. User needs to keep this lists name unchaged.
//
//*****************************************************************************

//******************************************************************************
//KWD Global data buffers used by ring buffers
//*****************************************************************************
    
//******************************************************************************
//KWD Global parameters
//*****************************************************************************
uint8_t g_ui8KwdDetectedFlag = 0;                   // 1: the key word is detected; 0: no key word detected.
uint8_t g_ui8ButtonDownFlag = 0;                    // 1: the button is pushed down; 0: no button is pushed.

uint8_t g_ui8PushTalkFlag = 0;                      // trig for AMA push-to-talk.
uint8_t g_ui8ProvideSpeechFlag = 0;                 // trig for AMA conversation talk.
#if configUSE_AUDIO_PRE_BUFF_AMA
uint8_t g_ui8KwdRequiredConsume = 0;
#endif

volatile uint32_t count = 0;
#if configUSE_RTT_RECORDER
uint8_t g_ui8RTTStartFlag = 0;                      // 1: the start of RTT recorder; 0: stop recording
uint32_t g_ui32RTTSendBytes = 0;
#endif

//Start pdm >100ms when power-up
uint16_t g_ui16WOSActiveFlag = 20;
//******************************************************************************
//KWD Global codec instance
//*****************************************************************************
#if configUSE_AUDIO_CODEC
    #if (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)
        sbc_t   g_BluezSBCInstance;
    #endif
#endif

//******************************************************************************
//KWD Global ring buffers
//*****************************************************************************
//am_app_utils_ring_buffer_t am_KWD_ring_buffers[AM_APP_RINGBUFF_MAX];
#define KWD_RINGBUFF_INIT_COUNT   0 //  (sizeof(g_KWD_RingBuffSetup)/sizeof(am_app_utils_ringbuff_setup_t))

