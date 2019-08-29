//*****************************************************************************
//
//! @file usr_include.h
//!
//! @brief Global includes for freertos_kwd.
//
//*****************************************************************************

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

#ifndef USR_INCLUDE_H
#define USR_INCLUDE_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
//#include "am_devices.h"
#include "am_util.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
//*****************************************************************************
//
// User hardware config include files.
//
//*****************************************************************************
#include "vole_board_config.h"
//#include "am_app_KWD_AWE.h"
//#include "am_app_KWD_ble.h"
#include "am_app_KWD_init.h"
//#include "am_app_KWD_rtt_recorder.h"
//*****************************************************************************

#if configUSE_AWE
#include "Errors.h"
#include "Framework.h"
#include "StandardDefs.h"
#include "PlatformTuningHandler.h"
#include "MathHelper.h"
#include "ControlDriver.h"
#include "PlatformAPI.h"
#endif
//*****************************************************************************
//
// Application Tasks include files.
//
//*****************************************************************************
//#include "am_app_KWD_isr.h"
//#include "am_app_KWD_task.h"
//#include "radio_task.h"
//*****************************************************************************
//
// User utils include files.
//
//*****************************************************************************
#include "am_app_utils.h"

//*****************************************************************************
//
// Codec include files.
//
//*****************************************************************************

#endif // FREERTOS_KWD_H
