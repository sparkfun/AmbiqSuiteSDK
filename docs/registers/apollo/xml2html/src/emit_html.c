/*******************************************************************************
**
**      File: emit_html.c
**
**    Description: This file emits an html file for each device.
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
 \file     emit_html.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/21/2002
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ioreg_device.h"



//////////////////////////////////////////////////////////////////////////////
// emit all of the bit field data for all bit fields in the bit table
// in the detailed per register view.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_per_reg_all_bf_in_bit_table(FILE *ofp, T_pIORegister pReg){
   T_pBitField pBfld;
   T_pBFEnum pBFEnum;
   pBfld = pReg->pBitField;
   while(pBfld != NULL){
      fprintf(ofp,"<TR align=\"left\">\n");
      fprintf(ofp,"<TD nowrap>%d:%d</TD>\n",
             (int)(pBfld->ulWidth+pBfld->ulLsb-1),(int)pBfld->ulLsb);
      fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->Name);
      fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->ReadWrite);
      if(pBfld->Interrupt != NULL){
        fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->Interrupt);
      } else if(pBfld->Reserved != NULL){
        fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->Reserved);
      } else if(pBfld->Empty != NULL){
        fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->Empty);
      } else if(pBfld->BackDoor != NULL){
        fprintf(ofp,"<TD nowrap>%s</TD>\n",pBfld->BackDoor);
      } else {
        fprintf(ofp,"<TD nowrap>-</TD>\n");
      } 
      fprintf(ofp,"<TD>%s",pBfld->Description);
      fprintf(ofp," <TABLE>\n");
      pBFEnum = pBfld->pBFEnum;
      while(pBFEnum !=NULL){
              fprintf(ofp,"<TR>\n");
              fprintf(ofp,"<TD><SMALL><TT>%s</TT></SMALL></TD>\n",pBFEnum->Name);
              fprintf(ofp,"<TD><SMALL><TT>=</TT></SMALL></TD>\n");
              fprintf(ofp,"<TD><SMALL><TT>%s</TT></SMALL></TD>\n",pBFEnum->Value);
              fprintf(ofp,"<TD width=\"5\"><SMALL><TT>-</TT></SMALL></TD>\n");
              fprintf(ofp,"<TD><SMALL><TT>%s</TT></SMALL></TD></TR>\n",pBFEnum->Description);

	      pBFEnum = pBFEnum->pBFEnumNext;
      }
      fprintf(ofp,"</TABLE>\n");
      fprintf(ofp,"</TD>");
      fprintf(ofp,"</TR>");
      pBfld = pBfld->pBitField;
   }
}


//////////////////////////////////////////////////////////////////////////////
// emit all of the bit fields in the full register view
// in the detailed per register view.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_per_reg_all_bf(FILE *ofp, T_pIORegister pReg){
   T_pBitField pBfld;
   fprintf(ofp,"<TR align=\"top\">\n");
   pBfld = pReg->pBitField;
   while(pBfld != NULL){
     if((pBfld->Reserved != NULL)&&( ! strcmp(pBfld->Reserved,"reserved"))){
       fprintf(ofp,"<TD bgcolor=\"#e0e0e0\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
             (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
     } else if( ! strcmp(pBfld->ReadWrite,"RO")){
       fprintf(ofp,"<TD bgcolor=\"#c0c0c0\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
             (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
     } else {
       fprintf(ofp,"<TD bgcolor=\"#ffffff\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
             (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
     }
     pBfld = pBfld->pBitField;
   }
   fprintf(ofp,"</TR>\n");
}

//////////////////////////////////////////////////////////////////////////////
// emit one register's worth of information in the detailed per register view
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_per_reg_one_reg(FILE *ofp, T_pIORegister pReg){
   int Instance;
   int i;
   fprintf(ofp,"<BR><HR>\n");
   fprintf(ofp,"<H2 id=\"%s\"><A href=\"#index\">\n", pReg->Name);
   fprintf(ofp,"%s_%s</A></H2>\n",
	   Device.DeviceName,pReg->Name);
   fprintf(ofp,"<P><FONT size=\"+1\"><B>%s</B></FONT></P>\n",pReg->Intro);
   fprintf(ofp,"<P>\n");
   fprintf(ofp,"<TABLE><TR>\n");
   fprintf(ofp,"<TD><B>REGISTER OFFSET:</B></TD>\n");
   fprintf(ofp,"<TD><TT>%s</TT></TD>\n",pReg->Offset);
   fprintf(ofp,"</TR>\n");
   if(Device.OnlyChild){
      fprintf(ofp,"<TR><TD><B> %s_%s ADDRESS:</B></TD>\n",
	      Device.DeviceName,pReg->Name);
      Instance = 0;
      fprintf(ofp,"<TD><TT>0x%8.8x</TT></TD></TR>\n",
              (unsigned int)(pReg->ulOffset + Device.ulInstanceBase[Instance]));
   } else {
     for( Instance =0;Instance<Device.InstanceCount;Instance++){ 
printf("emit_html_per_reg_one_reg: Instance not only child\n");
      fprintf(ofp,"<TR><TD><B> %s%d_%s ADDRESS:</B></TD>\n",
         Device.DeviceName,Instance, pReg->Name);
      fprintf(ofp,"<TD><TT>0x%8.8x</TT></TD></TR>\n",
         (unsigned int)(pReg->ulOffset + Device.ulInstanceBase[Instance]));
     }
   }
   fprintf(ofp,"</TABLE>\n");
   fprintf(ofp,"<TABLE border=\"1\" cellpadding=\"1\" cellspacing=\"0\">\n");
   fprintf(ofp,"<TR align=\"top\" bgcolor=\"#888888\">\n");
   for(i=31; i>=0; i--) fprintf(ofp,"<TD><SMALL><B>%d</B></SMALL></TD>\n",i);
   fprintf(ofp,"</TR>\n");

   emit_html_per_reg_all_bf(ofp, pReg);

   fprintf(ofp,"</TABLE>\n<BR/>\n");


   fprintf(ofp,"<TABLE border=\"1\" cellpadding=\"3\" cellspacing=\"0\">\n");
   fprintf(ofp,"<TR align=\"left\" bgcolor=\"#888888\">\n");
   fprintf(ofp,"<TH><U><SMALL>Bits</SMALL></U></TH>");
   fprintf(ofp,"<TH><U><SMALL>Name</SMALL></U></TH>");
   fprintf(ofp,"<TH><U><SMALL>RW</SMALL></U></TH>");
   fprintf(ofp,"<TH><U><SMALL>Flags</SMALL></U></TH>");
   fprintf(ofp,"<TH><U><SMALL>Description</SMALL></U></TH>\n");
   fprintf(ofp,"</TR>\n");

   emit_html_per_reg_all_bf_in_bit_table(ofp, pReg);

   fprintf(ofp,"</TABLE>\n");

   fprintf(ofp,"<P>\n");
   fprintf(ofp,"<B>Description:</B><BR>%s</P>\n",pReg->Description);
   fprintf(ofp,"<P>\n");
   fprintf(ofp,"<B>Examples:</B><TABLE bgcolor=\"#e0e0e0\" border=\"1\" cellpadding=\"5\" cellspacing=\"0\"><TR><TD><PRE>\n");
   fprintf(ofp,"%s\n",pReg->Example);
   fprintf(ofp,"</PRE></TD></TR></TABLE>\n </P>\n");
}

//////////////////////////////////////////////////////////////////////////////
// emit the leadin to the detailed view "per register"
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_per_reg_leadin(FILE *ofp){
   fprintf(ofp,"<BR><BR><HR>\n");

}

//////////////////////////////////////////////////////////////////////////////
// emit one bit field's worth of information within one row of the big 
// picture view of the registers.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_big_pix_one_bf(FILE *ofp, T_pIORegister pReg){
   T_pBitField pBfld;
   pBfld = pReg->pBitField;
   while(pBfld != NULL){
     if((pBfld->Reserved != NULL)&&( ! strcmp(pBfld->Reserved,"reserved"))){
       fprintf(ofp,"<TD bgcolor=\"#e0e0e0\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
              (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
       pBfld = pBfld->pBitField;
     } else if( ! strcmp(pBfld->ReadWrite,"RO")){
       fprintf(ofp,"<TD bgcolor=\"#c0c0c0\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
              (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
       pBfld = pBfld->pBitField;
     } else {
       fprintf(ofp,"<TD bgcolor=\"#ffffff\" colspan=\"%d\">%s<BR><TT>%s</TT>\n</TD>\n",
              (int)pBfld->ulWidth,pBfld->Name,pBfld->ResetValue);
       pBfld = pBfld->pBitField;
     }
   }
}

//////////////////////////////////////////////////////////////////////////////
// emit one register's row in the big picture table
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_big_pix_one_reg(FILE *ofp, T_pIORegister pReg){
   fprintf(ofp,"<TR align=\"top\">\n");
   fprintf(ofp,"<TH align=\"left\" bgcolor=\"#888888\">\n");
   fprintf(ofp,"<A href=\"#%s\"> \n",pReg->Name);
   fprintf(ofp,"%s_%s</A><BR>OFFSET: <TT>%s</TT>\n",
	   Device.DeviceName,pReg->Name,pReg->Offset);

   fprintf(ofp,"</TH>\n");

   emit_html_big_pix_one_bf(ofp, pReg);

}


//////////////////////////////////////////////////////////////////////////////
// emit the leadin to the big picture view of the registers.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_big_pix_leadin(FILE *ofp){
   int i;
   fprintf(ofp,"<BR><HR>\n");
   fprintf(ofp,"<H2 id=\"bigpicture\">%s Register Format Summary</H2>\n",
		   Device.DeviceName);
   fprintf(ofp,"<TABLE border=\"1\" cellpadding=\"1\" cellspacing=\"0\">\n");
   fprintf(ofp,"<TR align=\"top\" bgcolor=\"#888888\">\n");
   fprintf(ofp,"<TD><SMALL><B>Register</B></SMALL></TD>\n");
   for(i=31; i>=0; i--) fprintf(ofp,"<TD><SMALL><B>%d</B></SMALL></TD>\n",i);
   fprintf(ofp,"</TR>\n");


}

//////////////////////////////////////////////////////////////////////////////
// emit one line of the index
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_one_index(FILE *ofp, T_pIORegister pReg){
   fprintf(ofp,"<B><TT>%s: </TT></B><A id=\"%sindex\" href=\"#%s\"><B>%s_%s</B>\n",
		   pReg->Offset,pReg->Name,pReg->Name,
		   Device.DeviceName,pReg->Name,pReg->Name);
   fprintf(ofp," -%s Register</A><BR>\n",pReg->Name);

}

//////////////////////////////////////////////////////////////////////////////
// emit the lead in part of the HTML file.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html_leadin(T_pDevice pDev, FILE *ofp)
{
   int i;
   fprintf(ofp,"<HTML>\n<HEAD>\n");
   fprintf(ofp,"<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\">\n");
   fprintf(ofp,"<TITLE>IOXML FILE: %s %s %s</TITLE>\n","Apollo", pDev->DeviceName,pDev->ShortDescription);
   fprintf(ofp,"</HEAD>\n<BODY>\n <H1 align=\"center\">On-Line Documentation</H1>\n");
   //fprintf(ofp,"<FONT size=\"+2\"><B><P align=\"center\">%s</P></B><B><P align=\"center\">BUS TYPE: %s </P></B></FONT><HR>",
		   //pDev->ShortDescription,pDev->BusType);
   fprintf(ofp,"<FONT size=\"+2\"><B><P align=\"center\">%s</P></B></FONT><HR>",
		   pDev->ShortDescription,pDev->BusType);
   fprintf(ofp,"<FONT face=\"Helvetica\" size=\"-1\"><P align=\"center\"><B> Put banner announcement here. </B></P></FONT><HR>\n");
   fprintf(ofp,"<H2 id=\"index\"> %s Register Index</H2>\n",pDev->DeviceName);
   if(Device.OnlyChild){
       fprintf(ofp,"<P>\n<B> Device Base Address = </B><TT>0x%8.8x</TT>\n</P>\n",
               pDev->ulInstanceBase[i]);
   } else {
     for(i=0; i<pDev->InstanceCount;i++){
       fprintf(ofp,"<P>\n<B> Device Base Address[%d] = </B><TT>0x%8.8x</TT>\n</P>\n",
               i,pDev->ulInstanceBase[i]);
     }
   }
   fprintf(ofp,"<B> <TT>OFFSET</TT></B><BR>");
}

//////////////////////////////////////////////////////////////////////////////
// Entry point for emitting HTML files from xml register definitions.
//////////////////////////////////////////////////////////////////////////////
void 
emit_html(T_pDevice pDev, FILE *ofp)
{
   T_pIORegister pReg;
   //T_pBitField pBfld;
   //T_pBFEnum pBFEnum;

   emit_html_leadin(pDev,ofp);
   // build the register index register by register
   if(pDev->pRegister == NULL) return;
   pReg = pDev->pRegister;
   while(pReg != NULL){
     emit_html_one_index(ofp, pReg);
     pReg = pReg->pNext;
   }
   // build the big picture register by register
   emit_html_big_pix_leadin(ofp);
   pReg = pDev->pRegister;
   while(pReg != NULL){
     emit_html_big_pix_one_reg(ofp, pReg);
     pReg = pReg->pNext;
   }
   fprintf(ofp,"</TR>\n");
   fprintf(ofp,"</TABLE>\n");
   // build the per register view
   emit_html_per_reg_leadin(ofp);
   pReg = pDev->pRegister;
   while(pReg != NULL){
     emit_html_per_reg_one_reg(ofp, pReg);
     pReg = pReg->pNext;
   }
   fprintf(ofp,"<BR><BR><HR>\n");
   fprintf(ofp,"<FONT size=\"-2\"><P id=\"revision\">XML Source File Version Information</P>\n");
   fprintf(ofp,"<P><B>File Name:</B>%s</P>\n",pDev->FileName);
   fprintf(ofp,"<P><B>File Modification Date:</B>%s</P></FONT>\n",pDev->FileModDate);
   fprintf(ofp,"</BODY>\n</HTML>\n");

}
