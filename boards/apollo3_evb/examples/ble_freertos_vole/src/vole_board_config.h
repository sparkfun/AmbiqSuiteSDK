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
// This is part of revision 1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

///*******************************************************************************
//*
//*               Board hardware configurtion
//*               ---------------------------
//*
//********************************************************************************
//*     platform.h
//********************************************************************************
//*
//*     Description:  AWE Platform Interface Header File
//*
//*******************************************************************************/

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

// ---------------------------------------
// User Hardware Selection
// ---------------------------------------

/* Select an EVB Board */
//#define USE_APOLLO2_EVB
#define USE_APOLLO2_BLUE_EVB                       0//1
#define USE_APOLLO3_BLUE_EVB                       1//0
#define USE_MAYA                                   0//0    // Apollo3Blue Button board

// Select Mikro Board(s)used and MikroBUS slots
#define USE_VESPER_MIKRO_MB3                       0
#define USE_ST_MICS_MIKRO_MB3                      1
#define USE_EM9304_MIKRO_MB2                       0

/* Select Shield Board */
#define USE_MIKRO_DEV_SHIELD_REV1                  1

//*******************************************************************************
// System level functional module selection 
//*******************************************************************************

#define configUSE_SYSVIEWER             0
#define configUSE_SYS_LOG               0 
#define configUSE_RTT_RECORDER          0 
#define configUSE_STDIO_PRINTF          0

#define configUSE_AWE                   0               // AWE frame work switch
#define configUSE_MODEL_INFERENCE       1 

#define configUSE_SENSORY_THF_ONLY      0

#define configUSE_BLE                   1 
#define configUSE_AUDIO_CODEC           1

#if configUSE_BLE
#define USE_OUTPUT_AMVOS_AMA            1
#endif
//********************************************************************************
// AWE module configuration 
//********************************************************************************
#if configUSE_AWE
    #define configUSE_AWE_TUNING            0                           // AWE must be 1 if AWE_TUNING being set 1  
    #define configUSE_QSD                   1                           // enable QSD
    #define configUSE_Sensory_THF           1                           // DSPC and Sensory lib separated. (DSPC output -> Sensory input)
    #define configUSE_WOS                   0                           // enable wake on sound 
    #define configUSE_PUSH_TO_TALK          0
#endif

//****************************************
// AWE layout configuration selection 
//****************************************
#if configUSE_Sensory_THF
    #define configUSE_2CM_FBF_SCNR                      1
#else
    #define configUSE_2CM_FBF_SCNR_THF_AMA              0
    #define configUSE_2CM_FBF_SCNR_THF_AMA_25H          1
#endif

//********************************************************************************
// Audio Codec module configuration 
//********************************************************************************
#if configUSE_AUDIO_CODEC
    #define configUSE_SBC_BLUEZ             0               // Codec selection. You could only choose 1 
    #define configUSE_MSBC_BLUEZ            1  
#endif
//********************************************************************************
// BLE module configuration 
//********************************************************************************
#if configUSE_BLE
    #define configUSE_AUDIO_PRE_BUFF_AMA        1               // 500ms audio pre-buffering required by Amazon 
    #define configUSE_AUDIO_POST_BUFF           1
#endif

//********************************************************************************
// RTT recorder module configuration 
//********************************************************************************
#if configUSE_RTT_RECORDER
    #define configUSE_RECORD_RAW_PCM        0               // Slect which data be recorded
    #define configUSE_RECORD_FULL_FILTER    0 
    #define configUSE_RECORD_CODEC          0
#endif
//********************************************************************************
// System log module configuration 
//********************************************************************************
#if configUSE_SYS_LOG
    #define configUSE_LOG_UART0                 1               // This is mutex with AWE_TUNING and BLE 
    #define configUSE_LOG_UART1                 0
#endif

//********************************************************************************
// std IO sub module configuration 
//********************************************************************************
#if configUSE_STDIO_PRINTF
    #define configUSE_PRINTF_UART0          	0
    #define configUSE_PRINTF_RTT                0
    #define configUSE_PRINTF_SWO                1

    #define am_app_printf(...)                  am_app_utils_stdio_printf(__VA_ARGS__)

    #define AM_APP_LOG_DEBUG(...)               am_app_utils_stdio_printf(1, __VA_ARGS__)
    #define AM_APP_LOG_WARNING(...)             am_app_utils_stdio_printf(2, __VA_ARGS__)
    #define AM_APP_LOG_INFO(...)                am_app_utils_stdio_printf(3, __VA_ARGS__)

#else
    #define am_app_printf(...)
    #define AM_APP_LOG_DEBUG(...)               
    #define AM_APP_LOG_WARNING(...)             
    #define AM_APP_LOG_INFO(...)                

#endif

#if configUSE_WOS
    #define USE_WAKE_ON_SOUND                1
#endif

//********************************************************************************
// System Hyper-parameters Macro
//********************************************************************************

//********************************************************************************
// PCM parameters
//********************************************************************************
#define PCM_FRAME_SIZE                  80
#define PCM_FRAME_PUSH_OVER             3
#define PCM_SAMPLE_SIZE_IN_BITS         16
#define BYTES_PER_SAMPLE                4
#define PCM_SAMPLE_RATE_MS              16

//********************************************************************************
// AWE parameters
//********************************************************************************
#define AWE_FRAME_SIZE 					PCM_FRAME_SIZE
#define BYTES_PER_DSPC_SAMPLE           2
#define BYTES_PER_1MS_DSPC_SAMPLE       32
//********************************************************************************
// UART & System Log parameters
//********************************************************************************
#define UART0_BUFFER_SIZE               (512 * 2)       // gloabal UART queue size (used by transmit buffered service)
#define UART1_BUFFER_SIZE               (512 * 2)
#define UART_TRANSMIT_BUFFER            256             // size limit to frame added into global UART queue
// System message macro definition 
#define EMPTY_MESSAGE                   1
#define KEY_WORD_GOT_MESSAGE            0xaa111155

//********************************************************************************
// Audio pre & post buffer parameters
//********************************************************************************
#define BUFFERED_SAMPLES_COUNT          1600                            // 8 seconds timeout.
#define AUDIO_POSTBUFF_TIME_MS          (BUFFERED_SAMPLES_COUNT * 5)    // 8 seconds timeout.

#if configUSE_AUDIO_PRE_BUFF_AMA
#define AUDIO_PREBUFF_TIME_MS           500
#define AUDIO_BUFF_LEN_MS               2000
#else
#define AUDIO_PREBUFF_TIME_MS           1500 
#define AUDIO_BUFF_LEN_MS               500
#endif

//********************************************************************************
// Codec parameters
//********************************************************************************
#define AMA_BUFFER_SIZE                251
#define SBC_IN_RING_BUFF_SIZE           256
#define CODEC_MSBC_INPUT_SIZE          240
//
// If using SBC compression, select audio transfer compression ratio
// 1:1 = 256000 bps, 4:1 = 64000 bps, 8:1 = 32000 bps, 16:1 = 16000 bps
// mSBC has fixed output rate 57000 bps
//
//#define SBC_BLUEZ_COMPRESS_BPS          64000
//#define SBC_OUT_RING_BUFF_SIZE          (SBC_BLUEZ_COMPRESS_BPS / 1000)

#define MSBC_BLUEZ_COMPRESS_BPS         57000
#define CODEC_MSBC_OUTPUT_SIZE         (MSBC_BLUEZ_COMPRESS_BPS / 1000)

#define CODEC_OPUS_INPUT_SIZE     320 //320  in 16-bit(2 bytes) unit instead of 8-bit
#define CODEC_OPUS_OUTPUT_SIZE    80

//#if configUSE_SBC_BLUEZ
 //   #define CODEC_IN_RING_BUFF_SIZE     SBC_IN_RING_BUFF_SIZE
 //   #define CODEC_OUT_RING_BUFF_SIZE    SBC_OUT_RING_BUFF_SIZE

//#elif configUSE_MSBC_BLUEZ
    //#define CODEC_IN_RING_BUFF_SIZE     MSBC_IN_RING_BUFF_SIZE
 //   #define CODEC_MSBC_OUT_RING_BUFF_SIZE    MSBC_OUT_RING_BUFF_SIZE
//#else
 //   #define CODEC_IN_RING_BUFF_SIZE     0
 //   #define CODEC_OUT_RING_BUFF_SIZE    0
//#endif
//********************************************************************************
// BLE parameters
//********************************************************************************
#define BLE_MSBC_DATA_BUFFER_SIZE            CODEC_MSBC_OUTPUT_SIZE*2
#define BLE_OPUS_DATA_BUFFER_SIZE            CODEC_OPUS_OUTPUT_SIZE*2

//********************************************************************************
// RTT recorder parameters
//********************************************************************************
#define RTT_BUFFER_LENGTH               (128*1024)

///********************************************************************************
// Hardware Related Macro
//*********************************************************************************

#if (USE_APOLLO2_BLUE_EVB && USE_MIKRO_DEV_SHIELD_REV1)
    #define LED_SYSTEM                  29
    #define LED_D5                      29
    #define LED_D6                      14
    #define LED_D7                      15
    #define LED_D8                      10
    #define CTRL_BUTTON2                16
    #define CTRL_BUTTON3                18
    #define CTRL_BUTTON4                19
    #define PDM_CLK                     12
    #define PDM_CLK_PIN_CFG             AM_HAL_PIN_12_PDM_CLK
    #define PDM_DATA                    11
    #define PDM_DATA_PIN_CFG            AM_HAL_PIN_11_PDM_DATA

    #define UART0_MODULE                0
    #define UART1_MODULE                1 

    #if USE_VESPER_MIKRO_MB3
        #define WOS_MODE_PIN                4
        #define WOS_WAKE_PIN               19
    #endif
#elif (USE_APOLLO3_BLUE_EVB && USE_MIKRO_DEV_SHIELD_REV1)
    #define LED_SYSTEM                  17
    #define LED_D5                      14
    #define LED_D6                      15
    #define LED_D7                      30
    #define LED_D8                      10
    #define CTRL_BUTTON2                16
    #define CTRL_BUTTON3                18
    #define CTRL_BUTTON4                19
    #define PDM_CLK                     12
    #define PDM_CLK_PIN_CFG             AM_HAL_PIN_12_PDM_CLK
    #define PDM_DATA                    11
    #define PDM_DATA_PIN_CFG            AM_HAL_PIN_11_PDM_DATA

    #define UART0_MODULE                0
    #define UART1_MODULE                1
	
    #if USE_VESPER_MIKRO_MB3
//        #define WOS_MODE_PIN                4
//        #define WOS_WAKE_PIN               19
    #endif
#elif USE_MAYA

    #define LED_D1                      6
    #define LED_D2                      27
    #define LED_D3                      25
    #define LED_D4                      28
    #define LED_D5                      43
    #define LED_D6                      39
    #define LED_D7                      47
    #define LED_D8                      5
    #define CTRL_BUTTON2                29  // optional button, only with membrane switch
    #define CTRL_BUTTON3                40
    #define PDM_CLK                     12
    #define PDM_CLK_PIN_CFG             AM_HAL_PIN_12_PDM_CLK
    #define PDM_DATA                    11
    #define PDM_DATA_PIN_CFG            AM_HAL_PIN_11_PDM_DATA

    #define UART0_MODULE                0
    #define UART1_MODULE                1

    #define WOS_WAKE1_PIN               4
    #define WOS_MODE_PIN                49

#endif
/* Function declaration */
void am_app_KWD_board_init(void);


#endif
