//*****************************************************************************
//
//! @file compose_task.h
//!
//! @brief Functions and variables related to the compose task.
//!
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

#ifndef COMPOSE_TASK_H
#define COMPOSE_TASK_H

//*****************************************************************************
//
// Composition Settings
//
//*****************************************************************************
#define BACKGROUND_COLOR          0x80 // 0x00
#define BAND_COLOR                0xfc // white
#define COLOR_MAX                 0xfc
#define BAND_WIDTH                16

//*****************************************************************************
//
// Color Manipulation Macros
//
//*****************************************************************************
// Extract color from a 4p word
#define COL(word, pix)            (((word) >> ((pix) << 3)) & COLOR_MAX)
#define WRAP_COL(num, max)        (((num) > (max)) ? 0 : (num))
// Increment color
#define INC_COL(color, inc)       WRAP_COL((color) + (inc))
// 4 pixel worth of data
#define COLOR_4P(color)           ((color) | ((color) << 8) | ((color) << 16) | ((color) << 24))
// Increment color for each pixel
#define INC_COL_4P(w, inc)        (INC_COL(COL((w), 0), (inc)) | (INC_COL(COL((w), 1), (inc)) << 8) | (INC_COL(COL((w), 2), (inc)) << 16) | (INC_COL(COL((w), 3), (inc)) << 24))


//*****************************************************************************
//
// compose task handle.
//
//*****************************************************************************
extern TaskHandle_t compose_task_handle;
extern void         *g_PsramHandle;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void ComposeTaskSetup(void);
extern void ComposeTask(void *pvParameters);

#endif // COMPOSE_TASK_H
