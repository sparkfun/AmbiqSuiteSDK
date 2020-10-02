/*******************************************************************************
**
**      File: create_dirs.c
**
**    Description: This file contains a function to create all of the output
**                 directories used by scew_ioreg.
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
 \file     create_dirs.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     2/13/2002
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>




////////////////////////////////////////////////////////////////////////////
// create_one_dir  -- as its name implies ...
////////////////////////////////////////////////////////////////////////////
static int
create_one_dir(char *fname){
   struct stat statbuf;
   char temp[512];
   int rc;

   if(stat(fname, &statbuf) == 0) return 0; // directory exists
   //doesn't exist
   sprintf(temp,"mkdir %s",fname);
   rc = system(temp);
   if(rc) return rc; // mkdir failed?
   if(stat(fname, &statbuf) == 0) return 0; // directory now exists
   return -1; // still couldn't stat the directory
}



////////////////////////////////////////////////////////////////////////////
//  create_dirs creates ./rtl, ./c_incls, ./rtf, ./ghs
//  returns zero for success and non-zero for failures.
////////////////////////////////////////////////////////////////////////////
int
create_dirs(char *output_dir)
{
  int rc;
  char FName[512];
  char OutputDir[512];
  strcpy(OutputDir,output_dir);
  strcat(OutputDir,"/");


  sprintf(FName,"%s%s",OutputDir,"html");
  rc = create_one_dir(FName);
  if(rc) return rc;

  sprintf(FName,"%s%s",OutputDir,"rtf");
  rc = create_one_dir(FName);
  if(rc) return rc;

  sprintf(FName,"%s%s",OutputDir,"bin");
  rc = create_one_dir(FName);
  if(rc) return rc;

  return 0; //for all OK
}

