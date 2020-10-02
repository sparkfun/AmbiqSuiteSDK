/**************************************************************************
** File:string_store.c
**  
********************************************************************************
**  Copyright A6LABS, Inc. 2002
**************************************************************************/
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
 \file     string_store.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/4/2002
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRING_STORE_SIZE (1024*1024)
static FirstTime =1 ;
static char * pStringStore = NULL;
static int  CurrentStringStoreSize;
char * AddToStringStore(char *cp)
{
  int l;
  char * rcp;
  if(FirstTime){
    pStringStore=(char *)malloc(STRING_STORE_SIZE);
    if(pStringStore==NULL){
       fprintf(stderr,"StringStore: could not malloc %d\n",STRING_STORE_SIZE);
       exit(1);
    }
    CurrentStringStoreSize = STRING_STORE_SIZE;
    FirstTime = 0;
  }
  CurrentStringStoreSize -= (l=(strlen(cp)+1));
  if(CurrentStringStoreSize < 0){
    fprintf(stderr,"StringStore: out of string space\n\n");
    exit(2);
  }
  strcpy(pStringStore,cp);
  rcp = pStringStore;
  pStringStore = pStringStore + l;
  return rcp;
}
