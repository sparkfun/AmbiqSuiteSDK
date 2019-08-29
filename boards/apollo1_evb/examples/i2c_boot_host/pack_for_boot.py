#!/usr/bin/env python3

import argparse
import string

args = None

def main():
    global args
    args = get_arguments()

    with open(args.output + '.h', 'w') as f:
        print(print_template(), file=f)

def get_arguments():
    parser = argparse.ArgumentParser(description='Pack a raw binary into a C file that can be used with a boot host example')

    parser.add_argument('input', help='Input binary file')
    parser.add_argument('-o', dest='output', help='Output C file name')
    parser.add_argument('-e', dest='key', help='Input file is encrypted')
    parser.add_argument('-l', dest='link_address', help='Link Address of the input file.')

    return parser.parse_args()

def print_template():
    array = bin_to_array(args.input)

    # Define a set of names to be used in the output files.
    formatmap = {'size' : len(array),
                 'linkaddress' : '0x{:08X}'.format(int(args.link_address, 0)),
                 'crc' : '0x{:08X}'.format(crc32(array)),
                 'filename' : args.output,
                 'macroname' : args.output.upper(),
                 'varname' : args.output.title().replace('_', ''),
                 'image' : print_array(array)
                }

    if args.key is None:
        return c_template.substitute(formatmap)
    else:
        formatmap['key'] = int(args.key, 0)
        return encrypted_template.substitute(formatmap)


#******************************************************************************
#
# Manipulating binary data.
#
#******************************************************************************
def bin_to_array(filename):

    # Read in the whole file as a list of integers.
    with open(filename, mode='rb') as f:
        binarray = f.read()

    return binarray

def print_array(binarray):
    # S will be the string to accumulate the byte values in a C-friendly format
    S = '    '

    # Loop over binarray
    for n in range(len(binarray)):

        # Add the hex representation of each byte to S
        S += '0x{:02X}'.format(binarray[n])

        # Print in comma-separated lines, 8 bytes long each.
        if n % 8 == 7:
            S += ',\n    '
        else:
            S += ', '

    # The end of the previous loop will leave an extra comma and some amount of
    # whitespace after the last byte value. This line will remove that.
    S = S.rsplit(',', 1)[0]

    return S

#******************************************************************************
#
# CRC functions.
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

#******************************************************************************
#
# Templates
#
#******************************************************************************
c_template = string.Template('''
//*****************************************************************************
//
//! @file $filename.h
//!
//! @brief This is a generated file
//
//*****************************************************************************

#ifndef ${macroname}_H
#define ${macroname}_H

//*****************************************************************************
//
// Translation layer.
//
//*****************************************************************************
#define IMAGE_SIZE                          ${macroname}_SIZE
#define IMAGE_CRC                           ${macroname}_CRC
#define IMAGE_LINK_ADDRESS                  ${macroname}_LINK_ADDRESS
#define IMAGE_ARRAY                         g_pui8${varname}

//*****************************************************************************
//
// Image characteristics
//
//*****************************************************************************
#define ${macroname}_SIZE                    $size
#define ${macroname}_LINK_ADDRESS            $linkaddress
#define ${macroname}_CRC                     $crc

//*****************************************************************************
//
// Boot Image
//
//*****************************************************************************
const uint8_t g_pui8${varname}[$size]=
{
$image
};

#endif // ${macroname}_H
'''.strip())

encrypted_template = string.Template('''
//*****************************************************************************
//
//! @file $filename.h
//!
//! @brief This is a generated file
//
//*****************************************************************************

#ifndef ${macroname}_H
#define ${macroname}_H

//*****************************************************************************
//
// Image characteristics
//
//*****************************************************************************
#define ${macroname}_SIZE                   $size
#define ${macroname}_LINK_ADDRESS           $linkaddress
#define ${macroname}_CRC                    $crc
#define ${macroname}_KEY_NUMBER             $key

//*****************************************************************************
//
// Encryption information
//
//*****************************************************************************
const uint8_t ${varname}IV[16] =
{
    0x28, 0xAE, 0xD2, 0xA6, 0x2B, 0x7E, 0x15, 0x16,
    0x09, 0xCF, 0x4F, 0x3C, 0xAB, 0xF7, 0x15, 0x88
};

//*****************************************************************************
//
// Boot Image
//
//*****************************************************************************
const uint8_t g_pui8${varname}[$size]=
{
$image
};

#endif // ${macroname}_H
'''.strip())

if __name__ == '__main__':
    main()
