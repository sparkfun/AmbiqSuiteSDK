//*****************************************************************************
//
//! @file multi_boot_secure.c
//!
//! @brief Secure Bootloader Function hooks - sample implementation
//! This file contains a sample implementation of the customizable secure boot
//! function hooks
//!
//! It implements a simple XOR based encryption scheme in CBC mode.
//! These are not cryptographically secure algorithms & are implemented here
//! just as a reference. Real designs should replace them with design specific
//! cryptographic implementations (e.g. AES-CBC for Confidentiality, and SHA-HMAC
//! for signature authentication and integrity verification)
//!
//! Key Size is 4 Bytes, determined based on keyidx from a key bank
//!
//! Encryption:
//! CipherText[i] = ((PlainText[i] ^ IV[i]) ^ Key)
//! IV[i+1] = CipherText[i]
//!
//! Decryption:
//! PlainText[i] = ((CipherText[i] ^ Key) ^ IV[i])
//! IV[i+1] = CipherText[i]
//!
//! IV[0] is shared between Encryption and Decryption (in clear)
//!
//! Image Signature is computed as Key ^ ClearCRC
//!
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_bootloader.h"
#include "am_multi_boot.h"
#include "am_multi_boot_config.h"

#include "am_multi_boot_secure.h"

#define MAX_KEYS          8
#define KEY_SIZE_WORDS    1 // words

// Test only - key index 6 is invalid
uint32_t g_ui32KeyValid             = 0xFDFFFFFF;

// Location in flash where the key valid mask is maintained
// Set to a known value in the data segment just for test purpose
#define KEY_VALID_MASK_LOCATION     &g_ui32KeyValid

typedef struct
{
    // Area Authenticated by Signature - Begin
    uint32_t protection;
    uint32_t imageLength;
    // For validation of the image
    uint32_t imageSignature[KEY_SIZE_WORDS];
    uint32_t initVector[KEY_SIZE_WORDS];
    // Area Authenticated by Signature - End
} multiboot_security_info_t;

// This is the security trailer definition for this sample implementation
// This would be the trailing part of AM_BOOTLOADER_NEW_IMAGE message
typedef struct
{
    uint32_t keyIdx;
    multiboot_security_info_t secInfo;
    // Validate this message itself
    uint32_t signature[KEY_SIZE_WORDS];
} multiboot_security_trailer_t;

typedef struct
{
    uint32_t signature[KEY_SIZE_WORDS];
    uint32_t runningSignature[KEY_SIZE_WORDS];
    uint32_t imageLength;
    uint32_t version;
    uint32_t initVector[KEY_SIZE_WORDS];
    uint32_t key[KEY_SIZE_WORDS];
    uint32_t offset;
    uint32_t clearCRC;
} multiboot_secure_state_t;

static multiboot_secure_state_t g_sSecState;

static uint32_t keyTbl[MAX_KEYS][KEY_SIZE_WORDS] =
{
    {0xDEADBEEF, },
    {0xAAAAAAAA, },
    {0x11111111, },
    {0x00000000, },
    {0xFFFFFFFF, },
    {0x55555555, },
    {0xA5A5A5A5, },
    {0x66666666, },
};

static void multiboot_decrypt(uint32_t * data, uint32_t size_words, uint32_t *key, uint32_t key_size_words, uint32_t * iv)
{
    int i, j;
    uint32_t temp;
    for ( j = 0; j < size_words / key_size_words; j++ )
    {
        for ( i = 0; i < key_size_words; i++ )
        {
            temp = data[j * key_size_words + i];
            data[j * key_size_words + i] = (data[j * key_size_words + i] ^ key[i]) ^ iv[i];
            iv[i] = temp;
        }
    }
}

#if 0 // Not used - included here just for reference
static void multiboot_encrypt(uint32_t * data, uint32_t size_words, uint32_t *key, uint32_t key_size_words, uint32_t * iv)
{
    int i, j;
    for ( j = 0; j < size_words / key_size_words; j++ )
    {
        for ( i = 0; i < key_size_words; i++ )
        {
            data[j*key_size_words+i] = (data[j*key_size_words+i] ^ iv[i]) ^ key[i];
            iv[i] = data[j*key_size_words+i];
        }
    }
}
#endif

static bool
validate_keyidx(uint32_t keyIdx)
{
    if ( keyIdx > MAX_KEYS )
    {
        return true;
    }
    // Validate the key index to make sure it has not been revoked
    return am_bootloader_check_index(keyIdx, (uint32_t *)KEY_VALID_MASK_LOCATION);
}

static bool
validate_sec_trailer(multiboot_security_trailer_t *pTrailer)
{
    int i;
    // Validate signature of Seurity trailer
    // Signature = CRC32 of trailer ^ key
    uint32_t crc = 0;
    am_bootloader_partial_crc32(&(pTrailer->keyIdx), sizeof(pTrailer->keyIdx), &crc);
    am_bootloader_partial_crc32(&(pTrailer->secInfo), sizeof(pTrailer->secInfo), &crc);
    for ( i = 0; i < KEY_SIZE_WORDS; i++ )
    {
        if ( pTrailer->signature[i] != (crc ^ keyTbl[pTrailer->keyIdx][i]) )
        {
            return true;
        }
    }
    return false;
}

// Multiboot function hooks - sample implementation

// Verify the security trailer & initialize the security params
int
init_multiboot_secure(uint32_t length, uint32_t *pData,
                      bool bStoreInSram, am_bootloader_image_t *psImage,
                      uint32_t *pProtect)
{
    int i;
    multiboot_security_trailer_t *pSec = (multiboot_security_trailer_t *)pData;
    // Verify the length
    if ( length != sizeof(*pSec) )
    {
        return -1;
    }
    // Validate key index
    if ( validate_keyidx(pSec->keyIdx) )
    {
        return -1;
    }
    // Now Authenticate the security trailer itself
    if ( validate_sec_trailer(pSec) )
    {
        return -1;
    }
    // We could reject the download if we can not validate if before flashing
    // by checking bStoreInSram herea - optional
    // Initialize State Variables
    g_sSecState.imageLength = pSec->secInfo.imageLength;
    if ( g_sSecState.imageLength != psImage->ui32NumBytes )
    {
        return -1;
    }
    g_sSecState.clearCRC = 0;
    g_sSecState.offset = 0;
    for ( i = 0; i < KEY_SIZE_WORDS; i++ )
    {
        // Initialize IV
        g_sSecState.initVector[i] = pSec->secInfo.initVector[i];
        // Store Signature
        g_sSecState.signature[i] = pSec->secInfo.imageSignature[i];
        g_sSecState.runningSignature[i] = 0;
        g_sSecState.key[i] = keyTbl[pSec->keyIdx][i];
    }
    *pProtect = pSec->secInfo.protection; // Could be set if copy-protection is desired
    return 0;
}

void
multiboot_secure_decrypt(void *pData, uint32_t ui32NumBytes)
{
    // Do in place decryption
    // With knowledge of image structure, this can also grab the image version
    // from a fixed location from within image
    multiboot_decrypt((uint32_t *)pData, ui32NumBytes / 4,
        g_sSecState.key, KEY_SIZE_WORDS, g_sSecState.initVector);
    // Update the running CRC for clear image
    am_bootloader_partial_crc32(pData, ui32NumBytes, &g_sSecState.clearCRC);
    g_sSecState.offset += ui32NumBytes;
}

// Verify the authenticity and integrity of the Image
// return the clear CRC for integrity verification going forward
int
multiboot_secure_verify(uint32_t *pui32ClearCRC)
{
    int i;
    // Check the signature with computed Value - for authentication
    // Optionally check the version for replay protection
    // Return 0 for success
    // Check for image signature to validate authenticity (and integrity)
    // Image Signature is ClearCRC ^ key
    for ( i = 0; i < KEY_SIZE_WORDS; i++ )
    {
        g_sSecState.runningSignature[i] = g_sSecState.key[i] ^ g_sSecState.clearCRC;
        if ( g_sSecState.runningSignature[i] != g_sSecState.signature[i] )
        {
            return -1;
        }
    }
    // Update the CRC in psImage for integrity verification going forward
    *pui32ClearCRC = g_sSecState.clearCRC;
    return 0;
}

