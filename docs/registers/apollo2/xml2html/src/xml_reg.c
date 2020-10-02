/*******************************************************************************
**
**      File: xml_reg.c
**
**    Description: This file contains the main line program for the xml_rg
**                 processor.  It is based on the open source libxml2
**                 library.
**    NOTE: this work is derived from the scew_ioreg program licensed under GPL2 from A6LABS.
**
********************************************************************************
**  Copyright A6LABS, Inc. 2002
*******************************************************************************/
/*
 Copyright (c) 2002-2009 David Baker

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along with this program;
 if not, write to the Free Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*!
 \file     scew_ioreg.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/4/2002
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ioreg_device.h"

FILE *ofp;
FILE *html_ofp;
FILE *rtf_ofp;

////////////////////////////////////////////////////////////////////////////
//  MAIN ENTRY
////////////////////////////////////////////////////////////////////////////
int
main(int argc, char** argv)
{

    int rc;
    int Instance;
    char FName[512];
    char *output_dir = "/tmp";
     
    //ofp=fopen("trace.dat","w");
   
    if (argc < 2)
    {
        printf("usage: xml_reg regfile.xml [ouput_dir]\n");
        return EXIT_FAILURE;
    }
    if (argc > 2)output_dir = argv[2];

    //output_dir = "trial_run";

    //////////////////////////////////////////////////////
    // create any output directories that we will need
    //////////////////////////////////////////////////////
    if(create_dirs(output_dir)){
    //if(create_dirs("trial_run")){
        fprintf(stderr," one of the output directories could not be created\n");
        return EXIT_FAILURE;
    }
printf("running %s in directory %s\n",argv[1],output_dir);
    //////////////////////////////////////////////////////
    // parse the xml tree containing the device and register level 
    // information and generate the in memory device & registers.
    //////////////////////////////////////////////////////
    rc = ParseDeviceTree(argv[1]);
    if(rc){
        fprintf(stderr,"ParseDeviceTree rc = %d, exiting\n",rc);
        printf("ParseDeviceTree rc = %d, exiting\n",rc);
        return EXIT_FAILURE;
    }


    //////////////////////////////////////////////////////
    // emit an html file for the device
    //////////////////////////////////////////////////////
    sprintf(FName,"%s/html/%s_regs.htm",output_dir,Device.LowerDeviceName);
    printf("HTML: FNAME  <%s>\n",FName);
    html_ofp=fopen(FName,"w");
    if(html_ofp == NULL){
        fflush(stdout);
        fprintf(stderr,"could not open html file \n");
        fflush(stderr);
        return EXIT_FAILURE;
    }
    printf("\n\nHTML trial_run/html/%s.htm\n\n",Device.LowerDeviceName);
    emit_html(pDevice,html_ofp);
    printf("HTML trial_run/html/%s.htm\n",Device.LowerDeviceName);
    fclose(html_ofp);

    //////////////////////////////////////////////////////
    // emit a windows rtf file for the device
    //////////////////////////////////////////////////////
    //sprintf(FName,"trial_run/rtf/%s_regs.rtf",Device.LowerDeviceName);
    sprintf(FName,"%s/rtf/%s_regs.rtf",output_dir,Device.LowerDeviceName);
    rtf_ofp=fopen(FName,"w");
    if(rtf_ofp == NULL){
        fprintf(stderr,"could not open windows rtf file \n");
        return EXIT_FAILURE;
    }
    emit_rtf(pDevice,rtf_ofp);
    fclose(rtf_ofp);

#if 0
    sprintf(FName,"rtf/%s.rtf",Device.LowerDeviceName);

#endif


}
