#!/usr/bin/env python3

import argparse


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

#******************************************************************************
#
# Read in the binary files and output the merged binary
#
#******************************************************************************
def process(boot_loader_filename, app_filename,  output, loadaddr, overridegpio,
    overridepolarity, flagtype, flagaddr):

    # Open the file, and read it into an array of integers.
    with open(boot_loader_filename, mode = 'rb') as f_bl:
        boot_loader_binarray = f_bl.read()
        f_bl.close()

    # Open the file, and read it into an array of integers.
    with open(app_filename, mode = 'rb') as f_app:
        app_binarray = f_app.read()
        f_app.close()

    boot_length = len(boot_loader_binarray)
    flag_address = int(flagaddr, 16)
    print("boot size ",boot_length)
    # Make sure bootloader does not overlap with flag page
    if boot_length >= flag_address:
        print("ERROR boot loader image is too big");
        return

    app_length  = len(app_binarray)
    
    load_address = int(loadaddr, 16)
    pad_length = load_address - len(boot_loader_binarray)

    print("pad_length ",pad_length);

    # generate mutable byte array for the boot loader
    pad_binarray = bytearray([0]*pad_length);
    
    flag_type = int(flagtype, 16)
    
    # check flash flag storage type
    if flag_type == 0:
        # flag stored before application (usually 0x3c00)
    
        # this is where we will write the flash flag page info
        flag_page_location = flag_address - len(boot_loader_binarray)

        # Insert the application binary load address.
        print("load_address ",hex(load_address), "(",load_address,")")
        pad_binarray[flag_page_location + 0]  = (load_address >>  0) & 0x000000ff;
        pad_binarray[flag_page_location + 1]  = (load_address >>  8) & 0x000000ff;
        pad_binarray[flag_page_location + 2]  = (load_address >> 16) & 0x000000ff;
        pad_binarray[flag_page_location + 3]  = (load_address >> 24) & 0x000000ff;

        # put the application binary size into the padding array @ 0x3c04
        app_length  = len(app_binarray)
        print("app_size ",hex(app_length), "(",app_length,")")
        pad_binarray[flag_page_location + 4]  = (app_length >>  0) & 0x000000ff
        pad_binarray[flag_page_location + 5]  = (app_length >>  8) & 0x000000ff
        pad_binarray[flag_page_location + 6]  = (app_length >> 16) & 0x000000ff
        pad_binarray[flag_page_location + 7]  = (app_length >> 24) & 0x000000ff

        # compute the CRC for the application and write it to 0x3c08
        app_crc = crc32(app_binarray)
        print("crc =  ",hex(app_crc));
        pad_binarray[flag_page_location + 8]  = (app_crc >>  0) & 0x000000ff
        pad_binarray[flag_page_location + 9]  = (app_crc >>  8) & 0x000000ff
        pad_binarray[flag_page_location + 10] = (app_crc >> 16) & 0x000000ff
        pad_binarray[flag_page_location + 11] = (app_crc >> 24) & 0x000000ff

        # override gpio. default 0
        override_gpio = int(overridegpio, 16)
        pad_binarray[flag_page_location + 12] = (override_gpio >>  0) & 0x000000ff
        pad_binarray[flag_page_location + 13] = (override_gpio >>  8) & 0x000000ff
        pad_binarray[flag_page_location + 14] = (override_gpio >> 16) & 0x000000ff
        pad_binarray[flag_page_location + 15] = (override_gpio >> 24) & 0x000000ff

        # override polarity. default 0
        override_polarity = int(overridepolarity)
        pad_binarray[flag_page_location + 16] = (override_polarity >>  0) & 0x000000ff
        pad_binarray[flag_page_location + 17] = (override_polarity >>  8) & 0x000000ff
        pad_binarray[flag_page_location + 18] = (override_polarity >> 16) & 0x000000ff
        pad_binarray[flag_page_location + 19] = (override_polarity >> 24) & 0x000000ff

        # copy the reset vector stack pointer (SP) from the application binary to pad_binary
        pad_binarray[flag_page_location + 20] = app_binarray[0];
        pad_binarray[flag_page_location + 21] = app_binarray[1];
        pad_binarray[flag_page_location + 22] = app_binarray[2];
        pad_binarray[flag_page_location + 23] = app_binarray[3];

        # copy the reset vector program counter (PC) from the application binary to pad_binary
        pad_binarray[flag_page_location + 24] = app_binarray[4];
        pad_binarray[flag_page_location + 25] = app_binarray[5];
        pad_binarray[flag_page_location + 26] = app_binarray[6];
        pad_binarray[flag_page_location + 27] = app_binarray[7];
        
        # bEncrypted
        pad_binarray[flag_page_location + 28] = 0x00
        pad_binarray[flag_page_location + 29] = 0x00
        pad_binarray[flag_page_location + 30] = 0x00
        pad_binarray[flag_page_location + 31] = 0x00
        
        # CRC
        crc_flag = crc32(pad_binarray[flag_page_location:(flag_page_location + 32)])
        print("info crc =  ",hex(crc_flag));
        pad_binarray[flag_page_location + 32]  = (crc_flag >>  0) & 0x000000ff
        pad_binarray[flag_page_location + 33]  = (crc_flag >>  8) & 0x000000ff
        pad_binarray[flag_page_location + 34] = (crc_flag >> 16) & 0x000000ff
        pad_binarray[flag_page_location + 35] = (crc_flag >> 24) & 0x000000ff

        # now output all three binary arrays in the proper order
        with open(output + '.bin', mode = 'wb') as out:
            out.write(boot_loader_binarray)
            out.write(pad_binarray)
            out.write(app_binarray)
    else:
        # flag stored after application (usually last page of the internal flash, e.g. 0x7F800 for 512KB flash)
        page_length = 56;    #fixed page length
        
        #generate mutable byte array for the boot loader
        page_binarray = bytearray([0]*page_length);
        
        # Insert the application binary load address.
        print("load_address ",hex(load_address), "(",load_address,")")
        page_binarray[0]  = (load_address >>  0) & 0x000000ff;
        page_binarray[1]  = (load_address >>  8) & 0x000000ff;
        page_binarray[2]  = (load_address >> 16) & 0x000000ff;
        page_binarray[3]  = (load_address >> 24) & 0x000000ff;

        # put the application binary size into the padding array @ 0x3c04
        app_length  = len(app_binarray)
        print("app_size ",hex(app_length), "(",app_length,")")
        page_binarray[4]  = (app_length >>  0) & 0x000000ff
        page_binarray[5]  = (app_length >>  8) & 0x000000ff
        page_binarray[6]  = (app_length >> 16) & 0x000000ff
        page_binarray[7]  = (app_length >> 24) & 0x000000ff

        # compute the CRC for the application and write it to 0x3c08
        app_crc = crc32(app_binarray)
        print("crc =  ",hex(app_crc));
        page_binarray[8]  = (app_crc >>  0) & 0x000000ff
        page_binarray[9]  = (app_crc >>  8) & 0x000000ff
        page_binarray[10] = (app_crc >> 16) & 0x000000ff
        page_binarray[11] = (app_crc >> 24) & 0x000000ff

        # override gpio. default 0
        override_gpio = int(overridegpio, 16)
        page_binarray[12] = (override_gpio >>  0) & 0x000000ff
        page_binarray[13] = (override_gpio >>  8) & 0x000000ff
        page_binarray[14] = (override_gpio >> 16) & 0x000000ff
        page_binarray[15] = (override_gpio >> 24) & 0x000000ff

        # override polarity. default 0
        override_polarity = int(overridepolarity)
        page_binarray[16] = (override_polarity >>  0) & 0x000000ff
        page_binarray[17] = (override_polarity >>  8) & 0x000000ff
        page_binarray[18] = (override_polarity >> 16) & 0x000000ff
        page_binarray[19] = (override_polarity >> 24) & 0x000000ff

        # copy the reset vector stack pointer (SP) from the application binary to pad_binary
        page_binarray[20] = app_binarray[0];
        page_binarray[21] = app_binarray[1];
        page_binarray[22] = app_binarray[2];
        page_binarray[23] = app_binarray[3];

        # copy the reset vector program counter (PC) from the application binary to pad_binary
        page_binarray[24] = app_binarray[4];
        page_binarray[25] = app_binarray[5];
        page_binarray[26] = app_binarray[6];
        page_binarray[27] = app_binarray[7];
        
        # bEncrypted
        page_binarray[28] = 0x00
        page_binarray[29] = 0x00
        page_binarray[30] = 0x00
        page_binarray[31] = 0x00
        
        # CRC
        crc_flag = crc32(pad_binarray[flag_page_location:(flag_page_location + 32)])
        page_binarray[32]  = (crc_flag >>  0) & 0x000000ff
        page_binarray[33]  = (crc_flag >>  8) & 0x000000ff
        page_binarray[34] = (crc_flag >> 16) & 0x000000ff
        page_binarray[35] = (crc_flag >> 24) & 0x000000ff
        
        #generate a mutable fill array between application end and flag page start
        fill_binarray = bytearray([0xff]*(flag_address - app_length - load_address));
        
        # now output all three binary arrays in the proper order
        with open(output + '.bin', mode = 'wb') as out:
            out.write(boot_loader_binarray)
            out.write(pad_binarray)
            out.write(app_binarray)
            out.write(fill_binarray)
            out.write(page_binarray)
            
def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Combine Bootloader & main image in to a single download, with flag page.')

    parser.add_argument('--bootbin', dest='bootbin', default='../../../boards/apollo2_evb/examples/multi_boot/keil/bin/multi_boot.bin',
                        help='Bootloader binary file (multi_boot.bin)')

    parser.add_argument('--appbin', dest='appbin', default='../../../boards/apollo2_evb_em9304/examples/freertos_amota/keil/bin/freertos_amota.bin',
                        help='Application binary file (app.bin)')
            
    parser.add_argument('--load-address', dest='loadaddress', default='0x4000',
                        help='Load address of the application. Default = 0x4000 (for Apollo), set to 0x6000 for Apollo2')	

    parser.add_argument('--override-gpio', dest='overridegpio', default='0xffffffff',
                        help = 'Override GPIO number in hex. (Can be used to force a new image load)\n(0xffffffff to disable)')

    parser.add_argument('--override-polarity', dest='overridepolarity', default=0, type=int,
                        help = 'Polarity for the override pin.')

    #rma: add arg flag page address	
    parser.add_argument('--flag-type', dest='flagtype', default='0x0',
                        help = 'User specified flash flag page placement type. (0 = before application; 1 = after application)')	
    #rma: add arg flag page address	
    parser.add_argument('--flag-address', dest='flagaddress', default='0x3c00',
                        help = 'User specified flash flag page address, 0x3c00 as default (input 0x7f800 for last page of a 512KB flash Apollo device).')

    parser.add_argument('-o', dest = 'output', default = 'binary_array',
                        help = 'Output filename (without the extension)')

    args = parser.parse_args()

    return args

#******************************************************************************
#
# Main function.
#
#******************************************************************************
def main():
    # Read the arguments.
    args = parse_arguments()

    process(args.bootbin, args.appbin, args.output, args.loadaddress,
        args.overridegpio, args.overridepolarity,
        args.flagtype, args.flagaddress)

if __name__ == '__main__':
    main()
