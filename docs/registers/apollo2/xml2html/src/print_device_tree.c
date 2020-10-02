
/*******************************************************************************
**
**      File: print_device_tree.c
**
**    Description: This file contains device and register transversal.
**
**
**
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
 \file     print_device_tree.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/7/2002
*/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ioreg_device.h"


void 
print_device_tree(T_pDevice pDev, FILE *p_ofp)
{
   T_pIORegister pReg;
   T_pBitField pBfld;
   T_pBFEnum pBFEnum;
   fprintf(p_ofp,"\n\nDEVICE  INFORMATION:\n\n");
   fprintf(p_ofp,"DEVICE: %s -- %s\n",pDev->DeviceName,pDev->ShortDescription);
   fprintf(p_ofp,"\tBus Type: %s I/O Device Size %s\n",
                    pDev->BusType, pDev->IODeviceSize);
   fprintf(p_ofp,"\tFile Name : %s \n", pDev->FileName);
   fprintf(p_ofp,"\tFile Mod Date: %s\n", pDev->FileModDate);
   if(pDev->pRegister == NULL) return;
   pReg = pDev->pRegister;
   while(pReg != NULL){
       fprintf(p_ofp,"===============================================================================\n");
       fprintf(p_ofp,"print_device_tree: REGISTER\n");
       fprintf(p_ofp,"\tRegister Name: %s\n",pReg->Name);
       fprintf(p_ofp,"\tRegister Intro: %s\n",pReg->Intro);
       fprintf(p_ofp,"\tRegister Offset: %s\n",pReg->Offset);
       fprintf(p_ofp,"\tRegister Description: %s\n",pReg->Description);
       fprintf(p_ofp,"\tRegister Example: %s\n",pReg->Example);
       pBfld = pReg->pBitField;
       while(pBfld != NULL){
       fprintf(p_ofp,"\t\t========================================\n");
         fprintf(p_ofp,"\t\tBitfield name: %s\n",pBfld->Name);
         fprintf(p_ofp,"\t\tBitfield width: %s = %d\n",pBfld->Width,(int)pBfld->ulWidth);
         fprintf(p_ofp,"\t\tBitfield lsb: %s = %d\n",pBfld->Lsb,(int)pBfld->ulLsb);
         fprintf(p_ofp,"\t\tBitfield reset_value: %s = 0x%8.8x\n",pBfld->ResetValue,(int)pBfld->ulResetValue);
         fprintf(p_ofp,"\t\tBitfield read_write: %s\n",pBfld->ReadWrite);
         fprintf(p_ofp,"\t\tBitfield interrupt: %s\n",pBfld->Interrupt);
         fprintf(p_ofp,"\t\tBitfield reserved: %s\n",pBfld->Reserved);
         fprintf(p_ofp,"\t\tBitfield do_not_auto_test: %s\n",pBfld->DoNotAutoTest);
         fprintf(p_ofp,"\t\tBitfield empty: %s\n",pBfld->Empty);
         fprintf(p_ofp,"\t\tBitfield back_door: %s\n",pBfld->BackDoor);
         fprintf(p_ofp,"\t\tBitfield description: %s\n",pBfld->Description);
         pBFEnum = pBfld->pBFEnum;
	 while(pBFEnum !=NULL){
                 fprintf(p_ofp,"\t\t\t================================  ENUM\n");
                 fprintf(p_ofp,"\t\t\tBitfield enum name: %s\n",pBFEnum->Name);
                 fprintf(p_ofp,"\t\t\tBitfield enum value: %s = 0x%8.8x\n",pBFEnum->Value,(int)pBFEnum->ulValue);
                 fprintf(p_ofp,"\t\t\tBitfield enum description: %s\n",pBFEnum->Description);
		 pBFEnum = pBFEnum->pBFEnumNext;
	 }
         pBfld = pBfld->pBitField;
       }
       pReg = pReg->pNext;
   }
}
