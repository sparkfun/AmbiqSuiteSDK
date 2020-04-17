//*****************************************************************************
//
//! @file app_param.h
//!
//! @brief  Application framework parameter database for BT4 external flash.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
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
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

/*************************************************************************************************/
/*!
 *  \file   app_param.c
 *
 *  \brief  Application framework parameter database for BT4 external flash.
 *
 *          $Date: 2016-05-30 16:34:58 -0700 (Mon, 30 May 2016) $
 *          $Revision: 7286 $
 *
 *  Copyright (c) 2015 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include <stdio.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "bstream.h"
#include "app_param.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/**************************************************************************************************
  Global Variables
**************************************************************************************************/


/*************************************************************************************************/
/*!
 *  \fn     AppParamInit()
 *
 *  \brief  Initialize the parameter database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppParamInit(void)
{

}

/*************************************************************************************************/
/*!
 *  \fn     AppParamClear()
 *
 *  \brief  Clear the parameter database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppParamClear(void)
{

}


/*************************************************************************************************/
/*!
 *  \fn     AppParamWrite
 *
 *  \brief  Write parameter value.
 *
 *  \param  id          Identifier.
 *  \param  valueLen    Value length in bytes.
 *  \param  pValue      Value data.
 *
 *  \return Number of bytes written.
 */
/*************************************************************************************************/
uint16_t AppParamWrite(uint16_t id, uint16_t valueLen, const uint8_t *pValue)
{
    return 0;
}

/*************************************************************************************************/
/*!
 *  \fn     AppParamRead
 *
 *  \brief  Read parameter value.
 *
 *  \param  id          Identifier.
 *  \param  valueLen    Maximum value length in bytes.
 *  \param  pValue      Storage value data.
 *
 *  \return Number of bytes read.
 */
/*************************************************************************************************/
uint16_t AppParamRead(uint16_t id, uint16_t valueLen, uint8_t *pValue)
{
    return 0;
}
