/*********************************************************************************
**
**   FILE:ioreg_device.h 
**
**    This file contains the structure and typedefs for the memory representation
**    of the contents of the ioreg*.xml file.  This tree structure is generated
**    using scew and expat libraries to read the xml file.
**
********************************************************************************
**  Copyright A6LABS, Inc. 2002
*********************************************************************************/
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
 \file     ioreg_device.h
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/4/2002
*/


////////////////////////////////////////////////////////////////////
// Bit Field enumerator
////////////////////////////////////////////////////////////////////
typedef struct _bfenum_data_struct_ T_BFEnum, * T_pBFEnum;
struct _bfenum_data_struct_{
   T_pBFEnum pBFEnumNext; // point to next enumeration value
   char * Name;
   char * LowerName;
   char * Value;
   char * LowerValue;
   unsigned long   ulValue;
   char * Description;
};



////////////////////////////////////////////////////////////////////
// bit field data struct (obj). We use a linked list of these 
// off of each ioregister object.  first entry has bit 0. Last entry 
// has bit 31.
////////////////////////////////////////////////////////////////////
typedef struct _bitfield_data_struct_ T_BitField, * T_pBitField;
struct _bitfield_data_struct_{
  T_pBitField pBitField; // point to next bitfield, last one is NULL.
  char * Name;
  char * LowerName;
  char * Width;
  unsigned long   ulWidth;
  char * Lsb;
  unsigned long   ulLsb;
  char * ResetValue;
  unsigned long   ulResetValue; 
  char * ReadWrite;
  char * Interrupt;
  char * Reserved;
  char * DoNotAutoTest;
  char * Empty;
  char * SoftReset;
  char * BackDoor;
  char * BdSet;
  char * BdReset;
  char * Description;
  T_pBFEnum pBFEnum;
  T_pBFEnum pBFEnumTail;
};


////////////////////////////////////////////////////////////////////
// Register data struct (obj)  We use a linked list of these
////////////////////////////////////////////////////////////////////
typedef struct _ioregister_data_struct_ T_IORegisgter, * T_pIORegister;
struct _ioregister_data_struct_{
  T_pIORegister pNext;
  char * Name;
  char * LowerName;
  char * Intro;
  char * Offset;
  unsigned long ulOffset;
  char * Address;
  char * Description;
  char * Example;
  T_pBitField pBitField; // point to a list of bit fields, Last one is NULL
  T_pBitField pBitFieldTail; 
};



////////////////////////////////////////////////////////////////////
// device data struct (object)
////////////////////////////////////////////////////////////////////
typedef struct _device_data_struct_ *T_pDevice, T_Device;
struct _device_data_struct_{
   char * ShortDescription;
   char * BusType;
   char * DeviceName;
   char * LowerDeviceName;
   char * IODeviceSize;
   char * FileName;
   char * FileModDate;
   int    OnlyChild;
   int    InstanceCount;
   unsigned long    ulInstanceBase[4];
   T_pIORegister  pRegister,pRegisterTail;
};

extern T_Device   Device;
extern T_pDevice pDevice;
