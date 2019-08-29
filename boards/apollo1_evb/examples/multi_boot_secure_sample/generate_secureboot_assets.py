#!/usr/bin/env python3

import argparse
import sys
import os

# This key table has to match the one in bootloader
keyTbl = [0xDEADBEEF, 0xAAAAAAAA, 0x11111111, 0x00000000, 0xFFFFFFFF, 0x55555555, 0xA5A5A5A5, 0x66666666]

#******************************************************************************
#
# Main function
#
#******************************************************************************
def main():

    # Read the binary file from the command line.
    with open(args.binfile, mode='rb') as binfile:
        clear_application= binfile.read()

    print('Loading Clear application {} bytes from {}...'.format(len(clear_application), args.binfile), flush=True)
 
    plaintext = pad_to_block_size(clear_application, 4)

    ivVal = word_from_bytes(os.urandom(4), 0)
    print("Initialization Vector")
    print(hex(ivVal))
    application = encrypt_app(args.keyidxVal, plaintext, ivVal)
    trailer = sec_trailer(args.keyidxVal, plaintext, ivVal, int(args.protectionVal, 0))

    print('Saving encrypted image {} bytes to {}...'.format(len(application), args.encimagefile), flush=True)
    with open(args.encimagefile, mode='wb') as encimagefile:
        encimagebytearray = bytearray(application)
        encimagefile.write(encimagebytearray)

    print('Saving security trailer {} bytes to {}...'.format(len(trailer), args.sectrailerfile), flush=True)
    with open(args.sectrailerfile, mode='wb') as sectrailerfile:
        trailerbytearray = bytearray(trailer)
        sectrailerfile.write(trailerbytearray)

    print('Done.')

#******************************************************************************
#
# Turn a 32-bit number into a series of bytes for transmission.
#
# This command will split a 32-bit integer into an array of bytes, ordered
# LSB-first for transmission over the UART.
#
#******************************************************************************
def int_to_bytes(n):
    A = [n & 0xFF,
         (n >> 8) & 0xFF,
         (n >> 16) & 0xFF,
         (n >> 24) & 0xFF]

    return A

#******************************************************************************
#
# Extract a word from a byte array
#
#******************************************************************************
def word_from_bytes(B, n):
    return (B[n] + (B[n + 1] << 8) + (B[n + 2] << 16) + (B[n + 3] << 24))

#******************************************************************************
#
# CRC function that matches the CRC used by the Apollo bootloader.
#
#******************************************************************************
poly32 = 0x1EDC6F41
def crc32(L):
    rem = 0
    for b in L:
        rem = rem ^ (b << 24)
        for i in range(8):
            if rem & 0x80000000:
                rem = ((rem << 1) ^ poly32)
            else:
                rem = (rem << 1)

            rem = rem & 0xFFFFFFFF
    return rem


def pad_to_block_size(text, block_size):
    text_length = len(text)
    amount_to_pad = block_size - (text_length % block_size)
    if amount_to_pad == 0:
        amount_to_pad = block_size
    for i in range(0, amount_to_pad, 1):
        text += bytes(chr(amount_to_pad), 'ascii')
    return text

def encrypt_app(keyidx, clear_app, iv):
    key32 = keyTbl[keyidx]
    applen = len(clear_app)
    enc_app = []
    for i in range(0, applen, 4):
        word = word_from_bytes(clear_app, i)
        word = (word ^ iv) ^ key32
        iv = word
        enc_app.extend(int_to_bytes(word))
    return enc_app

def sec_trailer(keyidx, clear_app, iv, protection):
    key32 = keyTbl[keyidx]
    secTrailer = []
    secTrailer.extend(int_to_bytes(keyidx))
    secTrailer.extend(int_to_bytes(protection))
    applen = len(clear_app)
    secTrailer.extend(int_to_bytes(applen))
    crc = crc32(clear_app)
    sig = key32 ^ crc
    secTrailer.extend(int_to_bytes(sig))
    secTrailer.extend(int_to_bytes(iv))
    # Trailer Signature
    secTrailerSig = crc32(secTrailer) ^ key32
    secTrailer.extend(int_to_bytes(secTrailerSig))
    return secTrailer

#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description =
                                     'Secure Image generation utility for Apollo or Apollo2')
    parser.add_argument('binfile',
                        help = 'Binary file to program into the target device')

    parser.add_argument('keyidxVal', default=0, type=int, help = 'encryption key index')
 
    parser.add_argument('protectionVal', default=0, help = 'Image Protection Value (hex)')

    parser.add_argument('encimagefile', help = 'Destination file for Encrypted image')

    parser.add_argument('sectrailerfile', help = 'Destination file for security trailer')

    args = parser.parse_args()

    main()
