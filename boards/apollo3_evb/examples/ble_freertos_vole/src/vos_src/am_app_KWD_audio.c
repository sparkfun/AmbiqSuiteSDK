#include "usr_include.h"
//#include "am_app_KWD_AWE.h"
#include "stdint.h"
#include "vole_board_config.h"
#if configUSE_AWE
//#include "aweOptionalFeatures.h"
#endif

#if USE_OUTPUT_AMVOS_AMA
#include "am_app_KWD_AMA.h"
#endif

#if configUSE_SENSORY_THF_ONLY || configUSE_Sensory_THF
//#include "SensoryLib.h"
//#include "am_app_KWD_SensoryTHF.h"
#endif

int32_t g_delayedSample = 0;
int32_t g_endingSampleCnt = 0;

#if configUSE_AWE
//***********************************************************************
// Global AWE Instance
//***********************************************************************
AWEInstance g_AWEInstance;
CoreDescriptor g_Core[1];

//***********************************************************************
// Memory heaps
//***********************************************************************
AWE_FW_SLOW_ANY_CONST UINT32 g_master_heap_size = MASTER_HEAP_SIZE;
AWE_FW_SLOW_ANY_CONST UINT32 g_slow_heap_size = SLOW_HEAP_SIZE;
AWE_FW_SLOW_ANY_CONST UINT32 g_fastb_heap_size = FASTB_HEAP_SIZE;

#if configUSE_QSD
DSPC_QSD Q;
#endif

#if defined( __ICCARM__ )
    #pragma data_alignment=4
	UINT32 g_fastb_heap[FASTB_HEAP_SIZE] @ ".CCMRAM_Section";
    #pragma data_alignment=4
    UINT32 g_master_heap[MASTER_HEAP_SIZE];
    #pragma data_alignment=4
    UINT32 g_slow_heap[SLOW_HEAP_SIZE];
#else
	#if defined( __GNUC__ )
		__attribute__((__section__(".ccmram")))
	#else
        #if defined ( __CC_ARM )
		__attribute__((__section__(".data_CCMRAM")))
        #endif
	#endif
    __align(4) UINT32 g_fastb_heap[FASTB_HEAP_SIZE];
    __align(4) UINT32 g_master_heap[MASTER_HEAP_SIZE];
    __align(4) UINT32 g_slow_heap[SLOW_HEAP_SIZE];
#endif

/* ----------------------------------------------------------------------
** Module table
** ------------------------------------------------------------------- */

/* Array of pointers to module descriptors. This is initialized at compile time.
Each item is the address of a module descriptor that we need linked in. The
linker magic is such that only those modules referenced here will be in the
final program. */
const ModClassModule *g_module_descriptor_table[] =
{
	//------------- The suitably cast pointers to the module descriptors go here.
LISTOFCLASSOBJECTS

};

AWE_MOD_SLOW_DM_DATA UINT32 g_module_descriptor_table_size = sizeof(g_module_descriptor_table) / sizeof(g_module_descriptor_table[0]);

// Audio Running Flag

volatile BOOL g_bBlinkLED4ForBoardAlive = TRUE;

/** The only input pin for this core. */
static IOPinDescriptor s_InputPin[1];

/** The only output pin for this core. */
static IOPinDescriptor s_OutputPin[1];

volatile BOOL g_bAudioPump1Active = FALSE;
volatile BOOL g_bAudioPump2Active = FALSE;
volatile UINT32 g_nPumpCount = 0;

///-----------------------------------------------------------------------------
/// @name  INT32 awe_pltAudioStart(void)
/// @brief Start audio processing
///-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
INT32 awe_pltAudioStart(void)
{
	// At this point the model is fully instantiated and the control I/O can be setup
    ControlInit();

    return 0;

}   // End awe_pltAudioStart


///-----------------------------------------------------------------------------
/// @name  INT32 awe_pltAudioStop(void)
/// @brief Stop audio processing
///-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
INT32 awe_pltAudioStop(void)
{
    ControlDestroy();

    return 0;

}   // End awe_pltAudioStop

//-----------------------------------------------------------------------------
// METHOD:  awe_pltCoreSendCommand
// PURPOSE: Send command and wait for reply
//-----------------------------------------------------------------------------
INT32 awe_pltCoreSendCommand(void *pData, UINT32 * txBuffer, UINT32 * rxBuffer)
{
    return awe_fwPacketProcess(&g_AWEInstance);

}	// End awe_pltCoreSendCommand

//-----------------------------------------------------------------------------
// METHOD:  awe_pltGetCycleCount
// PURPOSE: Returns the current value in the counter
//-----------------------------------------------------------------------------
uint32_t awe_pltGetCycleCount(void)
{

#if USE_PROFILING
    uint32_t nCycles, nElapsedCycles;

    // read Stimer value for Ctimer Cycle count
    nCycles = am_hal_ctimer_read(0, AM_HAL_CTIMER_BOTH);

    // calculate how many milliseconds elapsed
    nElapsedCycles = nCycles << 2;  // (12Mhz clk source) multiply by 4

    return nElapsedCycles;
#else

    return NULL;
#endif

}   // End awe_pltGetCycleCount


//-----------------------------------------------------------------------------
// METHOD:  awe_pltElapsedCycles
// PURPOSE: Returns the cycle count between start time and end time
//-----------------------------------------------------------------------------
UINT32 awe_pltElapsedCycles(UINT32 nStartTime, UINT32 nEndTime)
{
#if USE_PROFILING
    UINT32 nElapsedTime;

    if (nEndTime > nStartTime)
    {
            nElapsedTime = nEndTime - nStartTime;
    }
    else
    {
        // Wrap around occurred
        nElapsedTime = ((((UINT32)0xFFFFFFFF) - nStartTime) + nEndTime + 1);
    }

	return nElapsedTime;
 #else

    return NULL;
#endif

}   // End awe_pltElapsedCycles




//***********************************************************************
// @ name:  am_app_KWD_AWE_instance_init
// @ brief: Initialize AWE Instance with target details
//***********************************************************************
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
void am_app_KWD_AWE_instance_init(void)
{
    uint32_t nInputWireInfo = INFO1_PROPS(INPUT_CHANNEL_COUNT,
                                          AWE_FRAME_SIZE,
                                          IS_COMPLEX,
                                          SAMPLE_SIZE_IN_BYTES);

    uint32_t nOutputWireInfo = INFO1_PROPS(OUTPUT_CHANNEL_COUNT,
                                         AWE_FRAME_SIZE,
                                         IS_COMPLEX,
                                         SAMPLE_SIZE_IN_BYTES);

    memset(&g_AWEInstance, 0, sizeof(AWEInstance) );

    // Point to the start and stop functions.
    g_AWEInstance.m_pAwe_pltAudioStart = awe_pltAudioStart;
    g_AWEInstance.m_pAwe_pltAudioStop = awe_pltAudioStop;

    // Point to our private pins.
    g_AWEInstance.m_pInterleavedInputPin = s_InputPin;
    g_AWEInstance.m_pInterleavedOutputPin = s_OutputPin;

    // Point to the global module table.
    g_AWEInstance.m_module_descriptor_table_size = g_module_descriptor_table_size;
    g_AWEInstance.m_pModule_descriptor_table = g_module_descriptor_table;

    // This will be core ID 0.
    g_AWEInstance.m_coreID = CORE_ID;

    awe_fwInitTargetInfo(&g_AWEInstance,
                     CORE_ID,
                     CORE_SPEED,
                     SAMPLE_SPEED,
                     "Apollo2",
                     PROCESSOR_TYPE_CORTEXM4,
                     HAS_FLOAT_SUPPORT,
                     HAS_FLASH_FILESYSTEM,
                     NO_HW_INPUT_PINS,
                     NO_HW_OUTPUT_PINS,
                     IS_SMP,
                     NO_THREADS_SUPPORTED,
                     FIXED_SAMPLE_RATE,
                     INPUT_CHANNEL_COUNT,
                     OUTPUT_CHANNEL_COUNT,
                     VER_DAY, VER_MONTH, VER_YEAR,
                     AWE_FRAME_SIZE,
                     MAX_COMMAND_BUFFER_LEN                   
                     );

    awe_fwInit_io_pins(&g_AWEInstance, 1);

    s_InputPin[0].sampleRate = FIXED_SAMPLE_RATE;
    s_InputPin[0].wireInfo1 = nInputWireInfo;
    s_InputPin[0].wireInfo3 |= CLOCK_MASTER_BIT;
    awe_SetPackedName(s_InputPin[0].m_pinName, "Input");

    s_OutputPin[0].sampleRate = FIXED_SAMPLE_RATE;
    s_OutputPin[0].wireInfo1 = nOutputWireInfo;
    awe_SetPackedName(s_OutputPin[0].m_pinName, "Output");

	// Allocate the heaps.
    g_AWEInstance.m_master_heap_size = MASTER_HEAP_SIZE;
    g_AWEInstance.m_slow_heap_size = SLOW_HEAP_SIZE;
    g_AWEInstance.m_fastb_heap_size = FASTB_HEAP_SIZE;

    g_AWEInstance.m_master_heap = g_master_heap;
    g_AWEInstance.m_slow_heap = g_slow_heap;
    g_AWEInstance.m_fastb_heap = g_fastb_heap;

    // Setup core configuration
    g_Core[0].m_pSendCommand = awe_pltCoreSendCommand;
    g_Core[0].m_coreID = CORE_ID;

    awe_pltDefineCores( (CoreDescriptor *)g_Core, NUM_CORES);
}   // End AWEInstanceInit
#endif

//uint32_t ui32TotalCompressNeed = 0;         // test only
#if configUSE_SENSORY_THF_ONLY || configUSE_Sensory_THF
    int16_t in16SensoryInputBuff[FRAME_LEN];
    uint16_t n16ThfIndex = 0;
#endif

//-----------------------------------------------------------------------------
// METHOD:  am_app_KWD_AWE_handler
// PURPOSE: Pass new samples to Audio Weaver for processing
//-----------------------------------------------------------------------------
#if defined (AM_PART_APOLLO3)
extern void *PDMHandle;
#endif // #if defined (AM_PART_APOLLO3)

#if configUSE_AWE
AWE_OPTIMIZE_FOR_SPEED
AWE_FW_FAST_CODE
#endif
void am_app_KWD_AWE_handler(int32_t *nLRSample)
{
#if configUSE_AWE
    uint32_t  fwInCount, fwOutCount;
    uint32_t  chan;
    int32_t   pinStride;
    uint32_t  layoutMask;
    UINT32  nComplete;
    int32_t * dest32BitPtr, * destLeftChannel32BitPtr, * destRightChannel32BitPtr;
    fract32*  src32BitPtr;
    int32_t n32BitSample;
    int16_t n16BitSample;

    const uint32_t nPinNdx = 0;
#endif
    
#if configUSE_SENSORY_THF_ONLY || configUSE_Sensory_THF
    errors_t result;
#endif

    //int32_t nSample;
    //int16_t in16AudioStreamBuff[AWE_FRAME_SIZE];

#if configUSE_AWE
    if (!awe_pltIsAudioRunning())
    {
        // Flush the PDM FIFO
#if defined (AM_PART_APOLLO3)
        am_hal_pdm_fifo_flush(PDMHandle);
#else
        am_hal_pdm_fifo_flush();
#endif //#if defined (AM_PART_APOLLO3)
    }
  else
  {
      nComplete = 0;

      // Get Current AWE layout number of channels
      awe_fwGetChannelCount(&g_AWEInstance, nPinNdx, &fwInCount, &fwOutCount);

    if (fwInCount > 0)
    {
      if (fwInCount >= 1)
        destLeftChannel32BitPtr = awe_fwGetInputChannelPtr(&g_AWEInstance, nPinNdx, 0, &pinStride);

      if (fwInCount >= 2)
        destRightChannel32BitPtr = awe_fwGetInputChannelPtr(&g_AWEInstance, nPinNdx, 1, &pinStride);

      for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
      {

        if (fwInCount >= 1)
        {
            *destLeftChannel32BitPtr = nLRSample[nSample] << 16;
            destLeftChannel32BitPtr += pinStride;
        }

        if (fwInCount >= 2)
        {
          *destRightChannel32BitPtr = nLRSample[nSample] & 0xFFFF0000;
          destRightChannel32BitPtr += pinStride;
        }
      }

      // Zero any non-HW AWE layout inputs
      for (chan = 2; chan < fwInCount; chan++)
      {
        // Determine where the Framework wants the input data written
        dest32BitPtr = awe_fwGetInputChannelPtr(&g_AWEInstance, nPinNdx, chan, &pinStride);

        for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
        {
          *dest32BitPtr = 0;
          dest32BitPtr += pinStride;
        }
      }
    }

//{{{

        for (chan = 0; chan < OUTPUT_CHANNEL_COUNT; chan++)
        {
            if (chan < fwOutCount)
            {
                // Determine where the Framework wants the Output data written
                src32BitPtr = (fract32 *)awe_fwGetOutputChannelPtr(&g_AWEInstance, nPinNdx, chan, &pinStride);
                for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
                {
                    n32BitSample = *src32BitPtr;

                    // Not using SPI, so must be using UART or BLE.. both operate on single bytes
                    // Audio Weaver has sample in high order bytes of 32-bit word
                    n16BitSample = n32BitSample >> PCM_SAMPLE_SIZE_IN_BITS;
                    
                    #if configUSE_Sensory_THF
                        in16SensoryInputBuff[n16ThfIndex + nSample] = n16BitSample;
                    #endif
                        
                    in16AudioStreamBuff[nSample] = n16BitSample;

                    src32BitPtr += pinStride;
                }
                #if configUSE_Sensory_THF
                    n16ThfIndex += AWE_FRAME_SIZE;
                    if(n16ThfIndex >= FRAME_LEN)
                    {
                        result = SensoryProcessBrick(in16SensoryInputBuff);
                        //n16ThfIndex = 0;
                        if(result == ERR_OK)
                        {
                            am_app_KWD_reset_detected_flag();
                            g_ui8KwdDetectedFlag = 1;
                            {
                                AUDIOINDEX epIndex, tailCount, stIndex;
                                appStruct_T *ap = &appStruct;
                                SensoryFindStartpoint(ap, &stIndex);
                                SensoryFindEndpoint(ap, &epIndex, &tailCount);
//                                AM_APP_LOG_INFO("ap->audioBufferLen = %d ap->audioGetIndex = %d, Startpoint index = %d\n", ap->audioBufferLen, ap->audioGetIndex, stIndex);
//                                AM_APP_LOG_INFO("ap->audioPutIndex = %d, Endpoint index = %d Estimated latency = %d samples\n", ap->audioPutIndex, epIndex, tailCount);

                                if(epIndex > stIndex)
                                    g_delayedSample = epIndex - stIndex + tailCount;
                                else
                                    g_delayedSample = ap->audioBufferLen - stIndex + epIndex + tailCount;
                                g_endingSampleCnt = tailCount;
                            }
                            #if configUSE_STDIO_PRINTF
                                AM_APP_LOG_INFO("[AM-VoS] Keyword Detected! \n");
                            #endif // #if configUSE_STDIO_PRINTF

                            am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
                                                    AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
                            xTimerStart(am_KWD_timers[AM_APP_TIMER_KWD_TIME_OUT], 0);                      
                        }
                        n16ThfIndex = 0;
                    }
                #endif               
                #if (configUSE_RTT_RECORDER && configUSE_RECORD_FULL_FILTER)
                    //
                    // Record the raw PCM data and send over RTT
                    //
                    if(g_ui8RTTStartFlag == 1)
                        am_app_KWD_rtt_record((void*)in16AudioStreamBuff, AWE_FRAME_SIZE*BYTES_PER_DSPC_SAMPLE); 
                
                #elif (configUSE_RTT_RECORDER && configUSE_RECORD_CODEC)
                                       
                    if(g_ui8RTTStartFlag == 1)
                    {
                        AM_CRITICAL_BEGIN;
                            am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_RTT_STREAM], 
                                                            in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
                        AM_CRITICAL_END;
                        
                        am_app_utils_task_send(AM_APP_TASK_AWE_PROCESSING, AM_APP_TASK_CODEC,
                                                            AM_APP_MESSAGE_LONG, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE, 
                                                            &am_KWD_ring_buffers[AM_APP_RINGBUFF_RTT_STREAM]);
                    }
                    else
                        //am_app_utils_flush_ring_buffer(&am_KWD_ring_buffers[AM_APP_RINGBUFF_RTT_STREAM]);
                #endif
                if (g_ui8KwdDetectedFlag || g_ui8PushTalkFlag || g_ui8ProvideSpeechFlag)
                {    
                #if configUSE_BLE
                    if(count == 0)
                    {
                        #if configUSE_AUDIO_PRE_BUFF_AMA
                            g_ui8KwdRequiredConsume = 1;
                        #endif

                        if (!g_ui8KwdDetectedFlag)
                            //am_app_utils_flush_ring_buffer(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);

                        if (!g_ui8ProvideSpeechFlag)
                            am_app_KWD_AMA_start_speech_send();
                    }
								   
                    count++;
					
                    if (count <= BUFFERED_SAMPLES_COUNT)
                    {

                      #if configUSE_AUDIO_POST_BUFF
                        AM_CRITICAL_BEGIN;
                        am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM], 
                                                            in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
                        AM_CRITICAL_END;
                        #if configUSE_AUDIO_CODEC
                            if (amvosIsConnected())
                                am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_CODEC,
                                                            AM_APP_MESSAGE_LONG, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE, 
                                                            &am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);
//                          ui32TotalCompressNeed += BYTES_PER_DSPC_SAMPLE * AWE_FRAME_SIZE;        // test only
						 
                        #endif
                                
                      #endif
                    }
                    else
                    {
                    	am_app_KWD_reset_detected_flag();
                        //am_app_utils_flush_ring_buffer(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);
                    }
                #else

                    #if configUSE_AUDIO_POST_BUFF
                        AM_CRITICAL_BEGIN;
                        am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM], 
                                                             in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
                        AM_CRITICAL_END;
					   
                        #if (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)
                            bQueueReValue = am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_CODEC,
                                                                     AM_APP_MESSAGE_LONG, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE, 
                                                                     &am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);
				         					 
                        #endif
												
                   #endif

                #endif /*configUSE_BLE*/
  
                }
                else
                {
                    #if 1 //configUSE_AUDIO_PRE_BUFF_AMA
                        AM_CRITICAL_BEGIN;
                        // push 16 bit data to the 8 bit Ring Buffer. If its full old data will be overwritten
                        am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM], 
                                                        in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
                        AM_CRITICAL_END;
                    #endif
                }
            }
        }
      
//}}}
    nComplete |= awe_fwAudioDMAComplete(&g_AWEInstance, nPinNdx, AWE_FRAME_SIZE);

    if (nComplete)
    {
        layoutMask = awe_fwEvalLayoutMask(&g_AWEInstance, NULL);

        if(layoutMask & 1)
        {
          if (!g_bAudioPump1Active)
          {
            g_bAudioPump1Active = TRUE;

            // If IDLE loop did not get some CPU time then skip the pump
            // g_nPumpCount gets reset in the idle loop method awe_pltTick
            awe_fwPump(&g_AWEInstance, 0);
            g_bAudioPump1Active = FALSE;
    //      am_hal_interrupt_pend_set(AM_HAL_INTERRUPT_SOFTWARE1);
          }
        }

        // If lower priority level processing ready pend an interrupt for it
        if (layoutMask & 2)
        {
          if (!g_bAudioPump2Active)
          {
     
            g_bAudioPump2Active = TRUE;
            
    //        am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_MODEL_INFERENCE,
    //                                AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
            // Fire the layout at user interrupt level
    //        awe_fwPump(&g_AWEInstance, 1);

    //        g_bAudioPump2Active = FALSE;
    //       am_hal_interrupt_pend_set(AM_HAL_INTERRUPT_SOFTWARE2);
          }
        }
    }

    // When this higher level priority method returns, processing will resume
    // with AudioWeaverPump_IRQ1 or AudioWeaverPump_IRQ2 if triggered.
    // If both were triggered then processing for AudioWeaverPump_IRQ2 will start
    // only when the processing for AudioWeaverPump_IRQ1 has completed
  }
#endif
#if configUSE_SENSORY_THF_ONLY
    AUDIOINDEX epIndex, tailCount, stIndex;
    appStruct_T *ap = &appStruct;

    for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
    {
          in16SensoryInputBuff[n16ThfIndex + nSample] = nLRSample[nSample] & 0xFFFF;
          in16AudioStreamBuff[nSample] = nLRSample[nSample] & 0xFFFF;
    }

    n16ThfIndex += AWE_FRAME_SIZE;

    if(n16ThfIndex >= FRAME_LEN)
    {
        result = SensoryProcessBrick(in16SensoryInputBuff);
        //n16ThfIndex = 0;
        if(result == ERR_OK)
        {
            am_app_KWD_reset_detected_flag();
            g_ui8KwdDetectedFlag = 1;

            SensoryFindStartpoint(ap, &stIndex);
            SensoryFindEndpoint(ap, &epIndex, &tailCount);
//            AM_APP_LOG_INFO("ap->audioBufferLen = %d ap->audioGetIndex = %d, Startpoint index = %d\n", ap->audioBufferLen, ap->audioGetIndex, stIndex);
//            AM_APP_LOG_INFO("ap->audioPutIndex = %d, Endpoint index = %d Estimated latency = %d samples\n", ap->audioPutIndex, epIndex, tailCount);

            if(epIndex > stIndex)
                g_delayedSample = epIndex - stIndex + tailCount;
            else
                g_delayedSample = ap->audioBufferLen - stIndex + epIndex + tailCount;
            g_endingSampleCnt = tailCount;

            #if configUSE_STDIO_PRINTF
                AM_APP_LOG_INFO("[AM-VoS] Keyword Detected! \n");
            #endif // #if configUSE_STDIO_PRINTF

            am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_LED,
                                    AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
            xTimerStart(am_KWD_timers[AM_APP_TIMER_KWD_TIME_OUT], 0);                      
        }
        n16ThfIndex = 0;
    }

    if (g_ui8KwdDetectedFlag || g_ui8PushTalkFlag || g_ui8ProvideSpeechFlag)
    {    
    #if configUSE_BLE
        if(count == 0)
        {
            #if configUSE_AUDIO_PRE_BUFF_AMA
                g_ui8KwdRequiredConsume = 1;
            #endif

            if (!g_ui8KwdDetectedFlag)
                //am_app_utils_flush_ring_buffer(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);

            if (!g_ui8ProvideSpeechFlag)
                am_app_KWD_AMA_start_speech_send();
        }
                                                       
        count++;
                            
        if (count <= BUFFERED_SAMPLES_COUNT)
        {

          #if configUSE_AUDIO_POST_BUFF
            AM_CRITICAL_BEGIN;
            am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM], 
                                                in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
            AM_CRITICAL_END;
            #if configUSE_AUDIO_CODEC
                if (amvosIsConnected())
                    am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_CODEC,
                                                AM_APP_MESSAGE_LONG, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE, 
                                                &am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);
//                          ui32TotalCompressNeed += BYTES_PER_DSPC_SAMPLE * AWE_FRAME_SIZE;        // test only
                                     
            #endif
                    
          #endif
        }
        else
        {
            am_app_KWD_reset_detected_flag();
            //am_app_utils_flush_ring_buffer(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM]);
        }
    #endif /*configUSE_BLE*/

    }
    else
    {
        AM_CRITICAL_BEGIN;
        // push 16 bit data to the 8 bit Ring Buffer. If its full old data will be overwritten
        am_app_utils_ring_buffer_push_fast(&am_KWD_ring_buffers[AM_APP_RINGBUFF_POST_AUDIO_STREAM], 
                                        in16AudioStreamBuff, BYTES_PER_DSPC_SAMPLE*AWE_FRAME_SIZE);
        AM_CRITICAL_END;
    }
#endif
}	// End AWEProcessing

void am_app_KWD_reset_detected_flag()
{
#if 0
    g_ui8KwdDetectedFlag = 0;
    g_ui8PushTalkFlag = 0;
    g_ui8ProvideSpeechFlag = 0;
    count = 0;
#endif
}

