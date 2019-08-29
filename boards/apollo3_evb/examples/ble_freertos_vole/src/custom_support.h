#ifndef CUSTOM_SUPPORT_H
#define CUSTOM_SUPPORT_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "opus.h"
#include "opus_defines.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define OVERRIDE_OPUS_ALLOC

#define OVERRIDE_OPUS_FREE

static OPUS_INLINE void *opus_alloc (size_t size)
{
   return pvPortMalloc(size);

}

static OPUS_INLINE void opus_free (void *ptr)
{
   vPortFree(ptr);
}


































#endif
