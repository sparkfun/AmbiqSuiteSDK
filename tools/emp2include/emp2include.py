#!/usr/bin/env python3

import argparse
import configparser
import os
import sys
import platform

#******************************************************************************
#
# Template for the output files
#
#******************************************************************************
hfile1 = '''
//*****************************************************************************
//
//! @file {name}.h
//!
//! @brief This is a generated file.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#ifndef {macroname}_H
#define {macroname}_H

//*****************************************************************************
//
// Length of the binary array in bytes.
//
//*****************************************************************************
#define {lengthmacro:35} {length}

// EM patch destination memory:
// 1 means EM patch will be programmed into OTP if emp file
// was not programmed before.
// 0 means EM patch will be programmed into IRAM each time when
// EM9304 is cold boot.

#define DEST_MEMORY_IRAM                    0
#define DEST_MEMORY_OTP                     1
#define {patch_dest_memory_macro:35} {dest_memory}

//*****************************************************************************
//
// EM9304 Container Info type
//
//*****************************************************************************
typedef struct
{{
  uint16_t	buildNumber;		// Firmware Build Number
  uint16_t  userBuildNumber;	// User defined Build Number (determines patch precedence)
  uint8_t	containerVersion;	// Container Version
  uint8_t	containerType;		// Container Type
  uint8_t	containerID;		// Container ID
  bool		applyPatch;			// Flag to apply this patch.
  uint8_t   startingPatch;		// Starting patch index.
  uint8_t	endingPatch;		// Ending patch index + 1.
}} em9304_container_info_t;

//*****************************************************************************
//
// Extracted binary array.
//
//*****************************************************************************
'''.strip()

hfile2 = '''

extern em9304_container_info_t g_p{varname}[{length}];
extern const uint8_t g_p{varname}HCICmd[{numpatches}][68];

#endif // {macroname}_H
'''.strip()

hfile3 = '''
//
// Note: A NULL patch was generated.  This means that there were no *.emp files
// processed during the generation of this file.
//
extern em9304_container_info_t g_p{varname}[1];
extern const uint8_t g_p{varname}HCICmd[1][68];

#endif // {macroname}_H
'''.strip()

cfile1 = '''
//*****************************************************************************
//
//! @file {name}.c
//!
//! @brief This is a generated file.
//
//*****************************************************************************

#include <stdint.h>
#include "{name}.h"

//*****************************************************************************
//
// Extracted binary array
//
//*****************************************************************************
'''.strip()

cfile2 = '''
em9304_container_info_t g_p{varname}[{length}] =
{{
//  Build  UserBuild Version  Type ID  bApply  Start End
'''.strip()

cfile2_null = '''
//
// This is a NULL patch.  It indicates that there were no *.emp files processed
// during the generation of this file.
//
em9304_container_info_t g_p{varname}[1] =
{{
  {{ 0xFFFF, 0xFFFF, 0xFF, 0xFF, 0xFF, false, 0x00, 0x00 }}
'''.strip()

cfile3 = '''
}};
const uint8_t g_p{varname}HCICmd[{numpatches}][68] =
{{
'''.strip()

cfile3_null = '''
}};
const uint8_t g_pEm9304PatchesHCICmd[1][68] =
{{
  {{ 0x00
  }}
'''.strip()



#******************************************************************************
#
# CRC32 lookup table.
#
#******************************************************************************
crc32Table = [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d ]

def crc32( previousCrc, L ):
	crc = previousCrc
	for b in L:
		crc = crc32Table[( crc ^ b) & 0xFF ] ^ ( crc >> 8 )
	return (~crc & 0xFFFFFFFF)
	
#******************************************************************************
#
# Read in the binary file and output a C array.
#
#******************************************************************************
def process(memory):
	print('Find *.emp files')
	# Change to the target directory with *.emp files.
	asps = []
	for subdir, dirs, files in os.walk('./'):
		for file in files:
			if file.endswith('.emp'):
				if subdir.endswith('./'):
					print(file)
					asps.append(file)

	output = 'em9304_patches'
	print(output)
	
	# S will be the string to accumulate the patch records in a C-friendly format
	S = ''
	
	# C will be the string to accumulate the patch HCI commands in a C-friendly format
	C = ''
		
	patchIndex = 0
	patchCount = 0
	
	# For each file in the list
	for file in asps:
		with open(file, mode = 'rb') as f:
			binarray = f.read()
			
		# Set the starting index
		cmdIndex = 0
		
		#print(len(binarray))
		while ((cmdIndex < len(binarray)) & ((len(binarray) - cmdIndex) > 20)):
			#print(cmdIndex)
			# Check for the start of a new container.
			if(binarray[cmdIndex:cmdIndex+4] == b'39me'):
				#print("Found container header")
				
				# Increment the patch count.
				patchCount += 1
				
				# Add a record start
				S += '  { '
			
				# Add the decimal representation of the patch Build Number Version
				S += '{0:4d}'.format(binarray[cmdIndex+12] + (binarray[cmdIndex+13] << 8))
				S += ', '

				# Add the decimal representation of the patch User Build Number
				S += ' {0:4d}'.format(binarray[cmdIndex+14] + (binarray[cmdIndex+15] << 8))
				S += ', '

				# Add the decimal representation of the patch Format Version
				S += '     {0:2d}'.format(binarray[cmdIndex+8])
				S += ', '

				# Add the decimal representation of the patch Container Type
				S += '    {0:2d}'.format(binarray[cmdIndex+9])
				S += ', '
				
				# Determine whether the patch should override user and must go into OTP.
				if ( (binarray[cmdIndex+9] == 0x09) or (binarray[cmdIndex+9] == 0x0A) ):
				    bOTPOverride = True
				else:
				    bOTPOverride = False

				# Add the decimal representation of the patch Container ID
				S += ' {0:2d}'.format(binarray[cmdIndex+10])
				S += ', '

				# Initialize the path application flag to false
				S += 'false, '
			
				# Set the starting patch index
				S += ' {0:3d}, '.format(patchIndex)
		
				cmdSize = binarray[cmdIndex+4] + (binarray[cmdIndex+5] << 8) + (binarray[cmdIndex+6] << 16) + (binarray[cmdIndex+7] << 24)
				#print(cmdSize)
				
				firstCmd = True
				seqNum = 1
		
				while (cmdSize > 0):
					# Add a record start
					C += '  { '
		
					# First record is EM_WritePatchStart HCI command
					if (firstCmd):
						# Determine the binary patch data length
						if (cmdSize <= 59):
							cmdLength = cmdSize
						else:
							cmdLength = 59
						# Add the HCI Command Header (Note: Set destination to IRAM1 until testing is complete)
						C += '0x27, 0xFC,\t\t// HCI Vendor Specific Command EM_WritePatchStart\n    '
						C += '0x{:02X},\t\t\t// HCI Parameter Length\n    '.format(cmdLength + 5) 
						if ( (memory == 'IRAM') and (not bOTPOverride) ):
							C += '0x00,\t\t\t// Destination Memory (0 = IRAM1; 1 = OTP)\n    '
						elif (memory == 'OTP'):
							C += '0x01,\t\t\t// Destination Memory (0 = IRAM1; 1 = OTP)\n    '
						else:
							print('ERROR in destination memory indication')
							return
						# Create the binary data to run the CRC across
						crcarray = binarray[cmdIndex:(cmdIndex+cmdLength)]
						# Clear the firstCmd flag
						firstCmd = False
						#Remaining records are EM_WritePatchContinue commands
					else:
						# Determine the binary patch data length
						if (cmdSize <= 58):
							cmdLength = cmdSize
						else:
							cmdLength = 58
					
						# Add the HCI Command Header (Note: Set destination to IRAM1 until testing is complete
						C += '0x28, 0xFC,\t\t// HCI Vendor Specific Command EM_WritePatchContinue\n    ' 
						C += '0x{:02X},\t\t\t// HCI Parameter Length\n    '.format(cmdLength + 6)
				
						# Add the Sequence Number
						C += '0x{:02X}, '.format(seqNum & 0xFF)
						C += '0x{:02X},\t\t\t// Sequence Number\n    '.format((seqNum >> 8) & 0xFF)
				
						# Create the binary data to run the CRC across
						crcarray = binarray[cmdIndex:(cmdIndex+cmdLength)]
				
						# Increment the Sequence Number
						seqNum = seqNum + 1
			
					# Add the CRC32 to the HCI command
					packetCRC = crc32(0xFFFFFFFF,crcarray)
					for i in range(0,4):
						C += '0x{:02X}, '.format(packetCRC & 0xFF)
						packetCRC = packetCRC >> 8
					C += '\t// CRC32'
					# Start a newline
					C += '\n    '
		
					# index
					i = 0
					# Loop over binarray
					for n in range(cmdIndex,(cmdIndex+cmdLength)):
						# Add the hex representation of each byte to S
						C += '0x{:02X}'.format(binarray[n])

						# Print in comma-separated lines, 8 bytes long each.
						if i % 8 == 7:
							C += ',\n    '
						else:
							C += ', '
						i += 1
				
					# The end of the previous loop will leave an extra comma and some amount of
					# whitespace after the last byte value. This line will remove that.
					C = C.rsplit(',', 1)[0]
					C += '\n  },\n'
		
					# Increment the patch index
					patchIndex += 1
					#print("End of While Loop")
					#print(patchIndex)
			
					# Adjust the command size
					cmdSize -= cmdLength
					#print(cmdSize)
					
					# Adjust the command index
					cmdIndex += cmdLength
					#print(cmdIndex)
					
				cmdIndex = (cmdIndex + 3) & 0xFFFFFFFC
				#print(cmdIndex)
				# Set the ending patch index
				S += '  {0:3d}'.format(patchIndex) + ' },\n'
			else:
				# skipping with extra bytes inserted by emp patch
				# tool
				cmdIndex += 1
    # The end of the previous loop will leave an extra comma and some amount of
    # whitespace after the last byte value. This line will remove that.
	C = C.rsplit(',', 1)[0]
	C += '\n};\n\n'

	# The end of the previous loop will leave an extra comma and some amount of
	# whitespace after the last byte value. This line will remove that.
	S = S.rsplit(',', 1)[0]
	
	if (memory == 'OTP'):
		memory_var = 1;
	else:
		memory_var = 0;

	if (patchIndex > 0):
		# Define a set of names to be used in the output files.
		formatmap = {'length' : patchCount,
                 'name' : output,
				 'numpatches' : patchIndex,
                 'macroname' : output.upper(),
                 'lengthmacro' : output.upper() + '_NUM_PATCHES',
                 'varname' : output.title().replace('_', ''),
                 'patch_dest_memory_macro' : output.upper() + '_DEST_MEMORY',
                 'dest_memory': memory_var,
                }
	else:
		# Define a set of names to be used in the output files.
		formatmap = {'length' : 1,
                 'name' : output,
				 'numpatches' : patchIndex,
                 'macroname' : output.upper(),
                 'lengthmacro' : output.upper() + '_NUM_PATCHES',
                 'varname' : output.title().replace('_', ''),
                 'patch_dest_memory_macro' : output.upper() + '_DEST_MEMORY',
                 'dest_memory': memory_var,
                }
	dir = os.path.join(os.getcwd(),'../../ambiq_ble/em9304')
	os.chdir(dir)
	with open(output + '.c', mode = 'w') as out:
		# Print the global fileheader.
		print(cfile1.format(**formatmap), file=out)
		if (patchIndex > 0):
			print(cfile2.format(**formatmap), file=out)	
			print(S, file=out)
			print(cfile3.format(**formatmap), file=out)
			print(C, file=out)
		else:
			print(cfile2_null.format(**formatmap), file=out)	
			print(S, file=out)
			print(cfile3_null.format(**formatmap), file=out)
			print(C, file=out)
		

	with open(output + '.h', mode = 'w') as out:
		print(hfile1.format(**formatmap), file=out)
		if (patchIndex > 0):
		    print(hfile2.format(**formatmap), file=out)
		else:
		    print(hfile3.format(**formatmap), file=out)

	return		
	
#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':
    print('Parse Args')
    parser = argparse.ArgumentParser(description =
                                     'Convert multiple EM9304 *.emp files to the em9304_patches.* files.')
    parser.add_argument('-o', dest = 'memory', default = 'IRAM',
                        help = 'Destination memory (IRAM or OTP)')

    args = parser.parse_args()
    print('Call function')
    process(args.memory)
