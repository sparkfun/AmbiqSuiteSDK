/*******************************************************************************
**
**      File: emit_rtf.c
**
**    Description: This file emits an rtf file for each device.
**
*******************************************************************************/
/*
 Copyright (c) 2010 James A. Baker

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
 \file     emit_rtf.c
 \author   James A. Baker (jabakera6@austin.rr.com)
 \date     12/9/2010
*/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ioreg_device.h"

const int NUM_BITS = 32;

////////////////////////////////////////////////////////////////////////////////
// David Baker's file copy routine for moving boilerplate pieces to target file
////////////////////////////////////////////////////////////////////////////////
static void emit_boilerplate(char * FName, FILE * ofp) {
	FILE *src_ofp;
	char temp[512];
	int i;
	src_ofp = fopen(FName,"r");
	if(src_ofp == NULL){
		perror(" could not open boiler plate file\n");
		fprintf(stderr," boiler plate file = %s\n",FName);
		exit(101);
	}
	fgets(temp,512,src_ofp);
	while( ! feof(src_ofp)){
		i = strlen(temp);
		if((i>2) && (temp[i-2] == '\r') && (temp[i-1]=='\n')){
			temp[i-2] ='\n';
			temp[i-1] ='\0';
		}
		fputs(temp,ofp);
		fgets(temp,512,src_ofp);
	}
	fclose(src_ofp);
}

////////////////////////////////////////////////////////////////////////////////
// emit the contents of the example paragraph
////////////////////////////////////////////////////////////////////////////////
static void emit_rtf_example_paragraph_strings(FILE *ofp, char *pCexample) {
	if(pCexample == NULL)return;
	if(strcmp(pCexample,"")==0)return;
#ifdef NOTDEF
	fprintf(ofp, "%s\n", pCexample);
#endif
	emit_boilerplate("../boilerplate/rtf_example_para_open.txt", ofp);
	while(*pCexample) {
		if(*pCexample == '\r'){++pCexample; continue;}
		if(*pCexample == '\t'){fprintf(ofp,"\\tx20\n"); ++pCexample; continue;}
		if(*pCexample == '\n') {
			if(*(pCexample+1) == '\r') {
				if(*(pCexample+2) == '\0'){++pCexample; continue;}
				else {
					fprintf(ofp, "\\par }\n");
					emit_boilerplate("../boilerplate/rtf_example_interpara.txt", ofp);
					++pCexample;
					continue;
				}
			}
			else {
				if(*(pCexample+1) != '\0') {
					fprintf(ofp, "\\par }\n");
					emit_boilerplate("../boilerplate/rtf_example_interpara.txt", ofp);
				}
				++pCexample;
				continue;
			}
		}
		fputc(*pCexample++, ofp);
	}
	emit_boilerplate("../boilerplate/rtf_example_endpara.txt", ofp);
}

////////////////////////////////////////////////////////////////////////////////
// One Bitfield of a Bitfield Description Table
////////////////////////////////////////////////////////////////////////////////
static void DescribeOneBitField(FILE *ofp, T_pIORegister pReg, T_pBitField pBfld) {
	T_pBFEnum pBFEnum;

	emit_boilerplate("../boilerplate/rtf_bitdesc_table_rowstart.txt", ofp);

	fprintf(ofp, "\\hich\\af39\\dbch\\af40\\loch\\f39 ");
	if(pBfld->ulWidth == 1) {
		fprintf(ofp, "%s : %s ", pBfld->Lsb, pBfld->Lsb);
	}
	else {
		fprintf(ofp, "%d : %s", (int)(pBfld->ulWidth + pBfld->ulLsb - 1), pBfld->Lsb);
	}
	fprintf(ofp, "\\cell \n");

	fprintf(ofp, "\\hich\\af39\\dbch\\af40\\loch\\f39 ");
	fprintf(ofp, "%s ", pBfld->Name);
	fprintf(ofp, "\\cell \n");

	fprintf(ofp, "\\hich\\af39\\dbch\\af40\\loch\\f39 ");
	fprintf(ofp, "%s ", pBfld->ReadWrite);
	fprintf(ofp, "\\cell \n");

	fprintf(ofp, "\\hich\\af39\\dbch\\af40\\loch\\f39 ");
	if(pBfld->Interrupt != NULL) {
		fprintf(ofp, "%s ", pBfld->Interrupt);
		fprintf(ofp, "\\par \n");
	}
	else if(pBfld->BackDoor != NULL) {
		fprintf(ofp, "%s ", pBfld->BackDoor);
		fprintf(ofp, "\\par \n");
	}
	else if(pBfld->Reserved != NULL) {
		fprintf(ofp, "%s ", pBfld->Reserved);
		fprintf(ofp, "\\par \n");
	}
	else if(pBfld->Empty != NULL) {
		fprintf(ofp, "%s ", pBfld->Empty);
		fprintf(ofp, "\\par \n");
	}
	else {
		fprintf(ofp, "-");
	}
	fprintf(ofp, "\\cell }\n");

	fprintf(ofp, "\\pard \\ltrpar \\s23\\ql ");
	fprintf(ofp, "\\li0\\ri0\\nowidctlpar\\noline\\intbl\\wrapdefault");
	fprintf(ofp, "\\hyphpar0\\aspalpha\\aspnum\\faauto\\adjustright\\rin0\\lin0 ");
	fprintf(ofp, "{\\rtlch\\fcs1 \\af40 \\ltrch\\fcs0 \\insrsid5061468 \\hich\\af39\\dbch\\af40\\loch\\f39 ");
	fprintf(ofp, "%s ", pBfld->Description);

	pBFEnum = pBfld->pBFEnum;
	if(pBFEnum != NULL) { // now add enum constants
		while(pBFEnum != NULL) {
			fprintf(ofp, "\\par }{\\rtlch\\fcs1 \\af2\\afs16 \\ltrch\\fcs0 \\f2\\fs16\\insrsid5061468 ");
			fprintf(ofp, "\\hich\\af2\\dbch\\af40\\loch\\f2 ");
			fprintf(ofp, "%s ", pBFEnum->Name);
			fprintf(ofp, " - %s ", pBFEnum->Value);
			fprintf(ofp, " : %s ", pBFEnum->Description);

			pBFEnum = pBFEnum->pBFEnumNext;
		}
	}

	// close out the description bit cell
	fprintf(ofp, "\\cell }\\pard\\plain \\ltrpar\\ql \n");
	emit_boilerplate("../boilerplate/rtf_bitdesc_table_rowend.txt", ofp);
}

////////////////////////////////////////////////////////////////////////////////
// Create the bitfield description table for a register
////////////////////////////////////////////////////////////////////////////////
static void CreateBitFieldDescriptionTable(FILE *ofp, T_pIORegister pReg) {
	T_pBitField pBfld;
	emit_boilerplate("../boilerplate/rtf_bitdesc_table_prologue.txt", ofp);
	emit_boilerplate("../boilerplate/rtf_bitdesc_table_header_cell.txt", ofp);

	// do layout bitfields
	pBfld = pReg->pBitField;
	while(pBfld != NULL) {
		DescribeOneBitField(ofp, pReg, pBfld);
		pBfld = pBfld->pBitField;
	}
	// layout bitfields finished

	emit_boilerplate("../boilerplate/rtf_bitdesc_table_epilogue.txt", ofp);
}

////////////////////////////////////////////////////////////////////////////////
// One Bitfield of a Bitfield Layout Table
////////////////////////////////////////////////////////////////////////////////
static void LayoutOneBitfield(FILE *ofp, T_pIORegister pReg, T_pBitField pBfld) {
	fprintf(ofp, "\\hich\\af39\\dbch\\af40\\loch\\f39 ");
	fprintf(ofp, "%s ", pBfld->Name);
	fprintf(ofp, "\\cell");
}

////////////////////////////////////////////////////////////////////////////////
// Create the bit layout table for a register
////////////////////////////////////////////////////////////////////////////////
static void CreateBitLayoutTable(FILE *ofp, T_pIORegister pReg) {
	T_pBitField pBfld;
	int bit_cellwidth = 311;
	emit_boilerplate("../boilerplate/rtf_bitlayout_table_prologue.txt", ofp);
	emit_boilerplate("../boilerplate/rtf_bitlayout_table_header_cell.txt", ofp);

	// generate first round row and cell formatting
	pBfld = pReg->pBitField;
	while(pBfld != NULL) {
		//do stuff
		fprintf(ofp, "\\clvertalt\\clbrdrt\\brdrtbl ");
		fprintf(ofp, "\\clbrdrl\\brdrs\\brdrw2\\brdrcf1 ");
		fprintf(ofp, "\\clbrdrb\\brdrs\\brdrw2\\brdrcf1 ");
		fprintf(ofp, "\\clbrdrr\\brdrtbl ");
		if((strcmp(pBfld->ReadWrite,"RO") == 0) && (pBfld->Reserved != NULL)) {
			fprintf(ofp, "\\clcbpat18");
		}
		fprintf(ofp, "\\cltxlrtb\\clftsWidth3\\clwWidth%d", bit_cellwidth);
		if((strcmp(pBfld->ReadWrite,"RO") == 0) && (pBfld->Reserved != NULL)) {
			fprintf(ofp, "\\clcbpatraw18 \\cellx%d", (int)(bit_cellwidth * (NUM_BITS - pBfld->ulLsb)));
		}
		else
			fprintf(ofp, " \\cellx%d", (int)(bit_cellwidth * (NUM_BITS - pBfld->ulLsb)));
		pBfld = pBfld->pBitField;
	}
	// first round row and cell formatting complete

	emit_boilerplate("../boilerplate/rtf_bitlayout_table_cellformat_mid.txt", ofp);

	// do layout bitfields
	pBfld = pReg->pBitField;
	while(pBfld != NULL) {
		LayoutOneBitfield(ofp, pReg, pBfld);
		pBfld = pBfld->pBitField;
	}
	// layout bitfields finished

	emit_boilerplate("../boilerplate/rtf_bitlayout_table_cellformat_end.txt", ofp);

	// generate second round row and cell formatting
	pBfld = pReg->pBitField;
	while(pBfld != NULL) {
		//do stuff
		fprintf(ofp, "\\clvertalt\\clbrdrt\\brdrtbl ");
		fprintf(ofp, "\\clbrdrl\\brdrs\\brdrw2\\brdrcf1 ");
		fprintf(ofp, "\\clbrdrb\\brdrs\\brdrw2\\brdrcf1 ");
		fprintf(ofp, "\\clbrdrr\\brdrtbl ");
		if((strcmp(pBfld->ReadWrite,"RO") == 0) && (pBfld->Reserved != NULL)) {
			fprintf(ofp, "\\clcbpat18");
		}
		fprintf(ofp, "\\cltxlrtb\\clftsWidth3\\clwWidth%d", bit_cellwidth);
		if((strcmp(pBfld->ReadWrite,"RO") == 0) && (pBfld->Reserved != NULL)) {
			fprintf(ofp, "\\clcbpatraw18 \\cellx%d", (int)(bit_cellwidth * (NUM_BITS - pBfld->ulLsb)));
		}
		else
			fprintf(ofp, " \\cellx%d", (int)(bit_cellwidth * (NUM_BITS - pBfld->ulLsb)));
		pBfld = pBfld->pBitField;
	}
	// second round row and cell formatting complete

	emit_boilerplate("../boilerplate/rtf_bitlayout_table_epilogue.txt", ofp);
}

////////////////////////////////////////////////////////////////////////////////
// Emit one register's worth of the RTF file.
////////////////////////////////////////////////////////////////////////////////
static void emit_rtf_one_reg_table(FILE *ofp, T_pIORegister pReg) {
	// create the bit layout table for this register
	CreateBitLayoutTable(ofp, pReg);

	// create the bit field description table
	CreateBitFieldDescriptionTable(ofp, pReg);
}

////////////////////////////////////////////////////////////////////////////////
// Emit one register's worth of the RTF file tables and paragraphs
////////////////////////////////////////////////////////////////////////////////
static void emit_rtf_one_reg_para(FILE *ofp, T_pIORegister pReg) {
	int Instance;

	emit_boilerplate("../boilerplate/rtf_descpara.txt", ofp);
	fprintf(ofp, "IO_%s_%s ", Device.DeviceName, pReg->Name);
	emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);
	fprintf(ofp, "%s ", pReg->Intro);
	emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);

	if(Device.OnlyChild) {
      Instance = 0;
		fprintf(ofp, "IO_%s_%s ADDRESS: 0x%8.8X\n", Device.DeviceName, pReg->Name,
			(unsigned int)(pReg->ulOffset + Device.ulInstanceBase[Instance]));
		//fprintf(ofp, "\\par }\n");
		emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);
	}
	else {
              for( Instance =0;Instance<Device.InstanceCount;Instance++){ 
			fprintf(ofp, "IO_%s%s_%s ADDRESS: 0x%8.8X\n", Device.DeviceName, Instance, pReg->Name,
				(unsigned int)(pReg->ulOffset + Device.ulInstanceBase[Instance]));
			emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);
              }
	}

	emit_rtf_one_reg_table(ofp, pReg);

	// add register description
	emit_boilerplate("../boilerplate/rtf_descpara.txt", ofp);
	fprintf(ofp, "DESCRIPTION:\n");
	emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);
	fprintf(ofp, "%s\n", pReg->Description);
	fprintf(ofp, "\\par }\n");

	// add register accessor macro examples
	emit_boilerplate("../boilerplate/rtf_descpara.txt", ofp);
	fprintf(ofp, "EXAMPLE(S):\n");
	emit_boilerplate("../boilerplate/rtf_desc_interpara.txt", ofp);
	emit_rtf_example_paragraph_strings(ofp, pReg->Example);
	fprintf(ofp, "\\par } \\pard\n");
}

////////////////////////////////////////////////////////////////////////////////
// Emit an RTF prologue.
////////////////////////////////////////////////////////////////////////////////
static void emit_rtf_prologue(T_pDevice pDev, FILE *ofp) {
	// put out the RTF prologue
	emit_boilerplate("../boilerplate/rtf_prologue.txt", ofp);
}

////////////////////////////////////////////////////////////////////////////////
// Emit an RTF epilogue.
////////////////////////////////////////////////////////////////////////////////
static void emit_rtf_epilogue(T_pDevice pDev, FILE *ofp) {
	fprintf(ofp, "}\n");
}

////////////////////////////////////////////////////////////////////////////////
// Entry point for emitting C include files from xml register definitions.
////////////////////////////////////////////////////////////////////////////////
void emit_rtf(T_pDevice pDev, FILE *ofp) {
	T_pIORegister pReg;
	T_pBitField pBfld;

printf("emit_rtf: STARTING\n");
	emit_rtf_prologue(pDev, ofp);

	// emit each register's paragraph and table content by walking the registers
	if(pDev->pRegister == NULL) return;
	pReg = pDev->pRegister;
	while(pReg != NULL) {
		emit_rtf_one_reg_para(ofp, pReg);
		pReg = pReg->pNext;
	}

	emit_rtf_epilogue(pDev, ofp);
}
