//*****************************************************************************
//
//! @file am_app_KWD_init.h
//!
//! @brief the header file of KWD system initialization functions
//!
//*****************************************************************************

//*****************************************************************************
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

#ifndef AM_APP_KWD_INIT
#define AM_APP_KWD_INIT

#include "vole_board_config.h"
#include "am_app_utils.h"
//#include "am_app_KWD_rtt_recorder.h"
#include "sbc.h"

#if USE_OUTPUT_AMVOS_AMA
#include "accessories.pb.h"
#endif

#if USE_MAYA
#define HEART_BEAT_PERIOD               pdMS_TO_TICKS(3000)
#else
#define HEART_BEAT_PERIOD               pdMS_TO_TICKS(1000)
#endif // #if USE_MAYA

#define RTT_RECORDER_RUNNING            pdMS_TO_TICKS(300)

#define KWD_TIME_OUT                    pdMS_TO_TICKS(AUDIO_POSTBUFF_TIME_MS)

#define GSENSOR_PERIOD                  pdMS_TO_TICKS(2000) 

#if configUSE_BLE
    extern sbc_t g_BluezSBCInstance;
#if USE_OUTPUT_AMVOS_AMA
    extern DeviceInformation g_amaDeviceInformation; 
#endif
#endif

extern uint8_t g_ui8KwdDetectedFlag;
extern uint8_t g_ui8ButtonDownFlag;                    // 1: the button is pushed down; 0: no button is pushed.
extern uint8_t g_ui8PushTalkFlag;
extern uint8_t g_ui8ProvideSpeechFlag;

#if configUSE_AUDIO_PRE_BUFF_AMA
extern uint8_t g_ui8KwdRequiredConsume;
#endif

extern volatile uint32_t count;

#if configUSE_RTT_RECORDER
    extern uint8_t g_ui8RTTStartFlag;             
    extern uint32_t g_ui32RTTSendBytes;
#endif

extern uint16_t g_ui16WOSActiveFlag;

extern void am_app_KWD_reset_detected_flag();
extern void am_app_KWD_board_init(void);
extern void am_app_KWD_WOS_pdm_enable(void);
extern void am_app_KWD_WOS_pdm_disable(void);

#endif
