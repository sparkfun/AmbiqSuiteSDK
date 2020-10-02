/*******************************************************************************
**
**      File: parse_device_tree.c
**
**    Description: This file contains the parser for the xml tree which generates
**                 the device and register structure in memory that is used by 
**                 all other operations in the ioreg program arsenal.
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
 \file     parse_device_tree.c
 \author   David C. Baker (dbaker@alumni.utexas.net)
 \date     1/4/2002
*/



#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libxml/tree.h>
#include <libxml/parser.h>

#include "ioreg_device.h"

#include "string_store.h"

  xmlDocPtr Doc;


  /* root of the device structure tree */
  T_Device   Device;
  T_pDevice pDevice  = &Device;

////////////////////////////////////////////////////////////////////////////
// utility to lower case a string, NOTE string can not be a constant
////////////////////////////////////////////////////////////////////////////
void my_lower_case_a_string(char string[] ){
   int i = 0;
   while(string[i]){
     string[i] = tolower(string[i]);
     i++;
   }
   
}



////////////////////////////////////////////////////////////////////////////
// debug print the xml parse tree
////////////////////////////////////////////////////////////////////////////
static void
print_element_names(int recursion, xmlNode * a_node)
{
  xmlNode *cur_node = NULL;
  for(cur_node = a_node; cur_node; cur_node = cur_node->next){
    if(cur_node->type == XML_ELEMENT_NODE) {
       switch(recursion){
        case  0: printf("==>");break;
        case  1: printf("1111==>");break;
        case  2: printf("\n====2222==>");break;
        case  3: printf("========3333==>");break;
        case  4: printf("============4444==>");break;
        case  5: printf("================5555==>");break;
        default: printf("default ========++++==========>");break;
       }
       printf("node_type: Element, name: %s\n",cur_node->name);
    }
    print_element_names(recursion+1,cur_node->children);
  }
}


////////////////////////////////////////////////////////////////////////////
// parse_bitfield  recursive descent parsing of bitfield & children
////////////////////////////////////////////////////////////////////////////
void 
parse_bitfield(xmlNode * a_node, T_pBitField pBitField)
{
    char    *      pString;
       printf("parse_bitfield @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
    if(a_node->type == XML_ELEMENT_NODE) {
       if( !strcmp("ioreg_bf_description",a_node->name)){
         printf("parse_bitfield: found bf description\n");
          pString = xmlNodeGetContent(a_node);
          pBitField->Description = AddToStringStore(pString);
          xmlFree(pString);
       }
    }

#if 0
    scew_element* child = NULL;
    scew_attribute* attribute = NULL;
    char *pAttrName; 
    char *pAttrValue;
    XML_Char const* contents = NULL;
    char *bitfld_child;
    T_pBFEnum pNewBFEnum;
    scew_list *list = NULL;

    bitfld_child = (char *)scew_element_name(element);
    if( ! strcmp("ioreg_bf_description",bitfld_child)){
      pBitField->Description = AddToStringStore( (char *)scew_element_contents(element));
    } else if( ! strcmp("ioreg_bf_enum",bitfld_child)){
      /**
       * Allocate a bit field structure and link it to ioreg structure.
       */
      pNewBFEnum = (T_pBFEnum)malloc(sizeof(struct _bfenum_data_struct_));
      if(pNewBFEnum == NULL){
        fprintf(stderr,"malloc %s %d\n",__FILE__,__LINE__);
        exit(1);
      }
      pNewBFEnum->pBFEnumNext = NULL;
      pNewBFEnum->Name = NULL;
      pNewBFEnum->Value = NULL;
      pNewBFEnum->ulValue = 0;
      if((char *)scew_element_contents(element) == NULL){
         fprintf(stderr," bad enumeration, no description for bf = <%s>\n",
            pBitField->Name);
         pNewBFEnum->Description =AddToStringStore( "Bad Description in XML file"); 
      } else {
        pNewBFEnum->Description =AddToStringStore( (char *)scew_element_contents(element)); 
      }
      if(pBitField->pBFEnumTail==NULL){// then first time
         pBitField->pBFEnum     = pNewBFEnum;
         pBitField->pBFEnumTail = pNewBFEnum;
      } else {
         pBitField->pBFEnumTail->pBFEnumNext = pNewBFEnum;
         pBitField->pBFEnumTail = pNewBFEnum;
      }
      /**
       * Iterates through the element's attribute list, collecting the
       * pair name-value.
       */
      list = scew_element_attributes(element);
      while (list != NULL)
      {
          attribute = scew_list_data(list);
          pAttrName   =(char *) scew_attribute_name(attribute);
          pAttrValue  =(char *) scew_attribute_value(attribute);
          if(! strcmp(pAttrName,"name")){
            pNewBFEnum->Name = AddToStringStore(pAttrValue);
            pNewBFEnum->LowerName = AddToStringStore(pAttrValue);
            my_lower_case_a_string(pNewBFEnum->LowerName);
          } else if(! strcmp(pAttrName,"value")){
            pNewBFEnum->Value = AddToStringStore(pAttrValue);
            pNewBFEnum->ulValue = strtoul(pNewBFEnum->Value,NULL,0);
            pNewBFEnum->LowerValue = AddToStringStore(pAttrValue);
            my_lower_case_a_string(pNewBFEnum->LowerValue);
          } else {
            fprintf(stderr,"ERROR: one of ioreg_bf_enum attributes is bad <%s>\n",bitfld_child);
            exit(1);
          }
          list = scew_list_next(list);
      }
    } else {
      fprintf(stderr,"ERROR: one of ioreg_bitfield element's children is bad <%s>\n",bitfld_child);
      exit(1);
    }
#endif

}


////////////////////////////////////////////////////////////////////////////
// get bitfield attributes
////////////////////////////////////////////////////////////////////////////
void 
get_bitfield_attributes(xmlNode * a_node, T_pBitField pBitField)
{
       printf("get_bitfield_attributes @%s:%d ",__FILE__,__LINE__);

       xmlChar *pString;
       pString = xmlGetProp(a_node,"name");
       if(pString != NULL){
         pBitField->Name = AddToStringStore(pString);
         pBitField->LowerName = AddToStringStore(pString);
         my_lower_case_a_string(pBitField->LowerName);
         printf("bit field name = <%s> <%s> \n  ", pBitField->Name, pBitField->LowerName);
       }
       xmlFree(pString);
       pString = xmlGetProp(a_node,"lsb");
       if(pString != NULL){
         pBitField->Lsb = AddToStringStore(pString);
         pBitField->ulLsb = strtoul(pBitField->Lsb,NULL,0);
         printf("bit LSB = <%s> 0x%x ", pBitField->Lsb,(unsigned int) pBitField->ulLsb);
       }
       xmlFree(pString);

       pString = xmlGetProp(a_node,"width");
       if(pString != NULL){
         pBitField->Width = AddToStringStore(pString);
         pBitField->ulWidth = strtoul(pBitField->Width,NULL,0);
         printf("bit Width = <%s> 0x%x ", pBitField->Width, (unsigned int)pBitField->ulWidth);
       }
       xmlFree(pString);

       pString = xmlGetProp(a_node,"readwrite");
       if(pString != NULL){
         pBitField->ReadWrite = AddToStringStore(pString);
         printf("ReadWrite = <%s> ", pBitField->ReadWrite);
       }
       xmlFree(pString);

       pString = xmlGetProp(a_node,"access_macro");
       if(pString != NULL){
         printf("\n  access_macro = <%s> ", pString);
       }
       xmlFree(pString);
       pString = xmlGetProp(a_node,"reserved");
       if(pString != NULL){
         pBitField->Reserved = AddToStringStore(pString);
         printf(" reserved = <%s> ", pBitField->Reserved);
       }
       xmlFree(pString);

       pString = xmlGetProp(a_node,"reset_value");
       if(pString != NULL){
         pBitField->ResetValue = AddToStringStore(pString);
         pBitField->ulResetValue = strtoul(pBitField->ResetValue,NULL,0);
         printf("reset_value = <%s> 0x%x ", pBitField->ResetValue,(unsigned int)pBitField->ulResetValue);
       }
       xmlFree(pString);
       printf("\n");


#if 0
    scew_list       *list = NULL;
    scew_attribute  *attribute = NULL;
    char            *pAttrName; 
    char            *pAttrValue;

    if (element != NULL)
    {
        /**
         * Iterates through the element's attribute list, printing the
         * pair name-value.
         */
        list = scew_element_attributes(element);
        while (list != NULL)
        {
            attribute = scew_list_data(list);
            pAttrName   =(char *) scew_attribute_name(attribute);
            pAttrValue  =(char *) scew_attribute_value(attribute);
            if(! strcmp(pAttrName,"name")){
              pBitField->Name = AddToStringStore(pAttrValue);
              pBitField->LowerName = AddToStringStore(pAttrValue);
              my_lower_case_a_string(pBitField->LowerName);
	    } else if(! strcmp(pAttrName,"width")){
              pBitField->Width = AddToStringStore(pAttrValue);
              pBitField->ulWidth = strtoul(pBitField->Width,NULL,0);
	    } else if(! strcmp(pAttrName,"lsb")){
              pBitField->Lsb = AddToStringStore(pAttrValue);
              pBitField->ulLsb = strtoul(pBitField->Lsb,NULL,0);
	    } else if(! strcmp(pAttrName,"reset_value")){
              pBitField->ResetValue = AddToStringStore(pAttrValue);
              pBitField->ulResetValue = strtoul(pBitField->ResetValue,NULL,0);
	    } else if(! strcmp(pAttrName,"readwrite")){
              pBitField->ReadWrite = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"interrupt")){
              pBitField->Interrupt = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"reserved")){
              pBitField->Reserved = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"do_not_autotest")){
              pBitField->DoNotAutoTest = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"empty")){
              pBitField->Empty = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"backdoor")){
              pBitField->BackDoor = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"bd_set")){
              pBitField->BdSet = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"bd_reset")){
              pBitField->BdReset = AddToStringStore(pAttrValue);
	    } else if(! strcmp(pAttrName,"softreset")){
              pBitField->SoftReset = AddToStringStore(pAttrValue);
            } else {
              fprintf(stderr,"ERROR: one of ioreg_bf_enum attributes is bad <%s>\n",pAttrName);
              exit(1);
            }
            list = scew_list_next(list);
        }
    }
#endif
}


////////////////////////////////////////////////////////////////////////////
// parse_encoding  recursive descent parsing of register encoding & children
////////////////////////////////////////////////////////////////////////////
void 
parse_encoding(xmlNode * a_node, T_pIORegister  pRegister)
{
    T_pBitField    pNewBfld;
    xmlNode *      cur_node;
    if(a_node->type == XML_ELEMENT_NODE) {
       //printf("parse encoding @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
       if( !strcmp("ioreg_bitfield",a_node->name)){
          printf("parse encoding found bit_field @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
          /**
           * Allocate a bit field structure and link it to ioreg structure.
           */
          pNewBfld = (T_pBitField)malloc(sizeof(struct _bitfield_data_struct_));
          if(pNewBfld == NULL){
            fprintf(stderr,"malloc %s %d\n",__FILE__,__LINE__);
            exit(1);
          }
          pNewBfld->pBitField = NULL;
          pNewBfld->pBFEnum = NULL;
          pNewBfld->pBFEnumTail = NULL;
          pNewBfld->Name = NULL;
          pNewBfld->Width = NULL;
          pNewBfld->ulWidth = 0;
          pNewBfld->Lsb = NULL;
          pNewBfld->ulLsb = 0;
          pNewBfld->ResetValue = NULL;
          pNewBfld->ReadWrite = NULL;
          pNewBfld->Interrupt = NULL;
          pNewBfld->Reserved = NULL;
          pNewBfld->DoNotAutoTest = NULL;
          pNewBfld->Empty = NULL;
          pNewBfld->BackDoor = NULL;
          pNewBfld->BdSet = NULL;
          pNewBfld->BdReset = NULL;
          pNewBfld->SoftReset = NULL;
          pNewBfld->Description = NULL;
          if(pRegister->pBitFieldTail==NULL){// then first time
             pRegister->pBitField     = pNewBfld;
             pRegister->pBitFieldTail = pNewBfld;
          } else {
             pRegister->pBitFieldTail->pBitField = pNewBfld;
             pRegister->pBitFieldTail = pNewBfld;
          }
          get_bitfield_attributes(a_node,pNewBfld);
          /**
           * recursive descent parsing.
           * Call parse_bitfield function for each child of the
           * current element.
           */
          for(cur_node = a_node->children; cur_node; cur_node = cur_node->next){
              parse_bitfield(cur_node,pNewBfld);
          }

       } else {
         fprintf(stderr,"ERROR: one of ioreg_encoding element's children is bad @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
         exit(1);
       }
    }

#if 0
    scew_element   *child = NULL;
    scew_list      *list = NULL;
    XML_Char const *contents = NULL;
    char           *bitfld_child;
    T_pBitField    pNewBfld;

    bitfld_child = (char *)scew_element_name(element);
    if( ! strcmp("ioreg_bitfield",bitfld_child)){
      get_bitfield_attributes(element,pNewBfld);
      /**
       * recursive descent parsing.
       * Call parse_bitfield function for each child of the
       * current element.
       */
      list = scew_element_children(element);
      while (list != NULL)
      {
          child = scew_list_data(list);
          parse_bitfield(child,pNewBfld);
          list = scew_list_next(list);
      }
    } else {
      fprintf(stderr,"ERROR: one of ioreg_encoding element's children is bad <%s>\n",bitfld_child);
      exit(1);
    }
#endif

}

////////////////////////////////////////////////////////////////////////////
// parse_ioregisters  recursive descent parsing of ioreg & children
////////////////////////////////////////////////////////////////////////////
void 
parse_ioreg(xmlNode * a_node, T_pIORegister  pRegister)
{
    char    *      pString;
    xmlNode *      cur_node;
    if(a_node->type == XML_ELEMENT_NODE) {
       //printf("parse one ioregister @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
       if( !strcmp("ioreg_name",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          pRegister->Name = AddToStringStore(pString);
          pRegister->LowerName = AddToStringStore(pString);
          my_lower_case_a_string(pRegister->LowerName);
          xmlFree(pString);
          printf("\n\nfound ioreg_name <%s> <%s>\n", pRegister->Name,pRegister->LowerName);
       } else if( !strcmp("ioreg_offset",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          pRegister->Offset = AddToStringStore(pString);
          pRegister->ulOffset = strtoul(pRegister->Offset,NULL,0);
          xmlFree(pString);
          printf("found ioreg_offset<%s> 0x%x\n", pRegister->Offset,(unsigned int)pRegister->ulOffset);
       } else if( !strcmp("ioreg_intro",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          pRegister->Intro = AddToStringStore(pString);
          xmlFree(pString);
          printf("found ioreg_intro<%s>\n", pRegister->Intro);
       } else if( !strcmp("ioreg_description",a_node->name)){
          printf("found ioreg_description\n");
          pString = xmlNodeGetContent(a_node);
          if(pString == NULL){
             fprintf(stderr," bad register def, no description for reg = <%s>\n",
                pRegister->Name);
            pRegister->Description = AddToStringStore( "No register description in XML file");
          } else {
           pRegister->Description = AddToStringStore(pString);
          }
          xmlFree(pString);
          printf("found ioreg_description<%s> \n", pRegister->Description);
       } else if( !strcmp("ioreg_example",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          pRegister->Example = AddToStringStore(pString);
          printf("found ioreg_example<%s>\n", pRegister->Example);
          xmlFree(pString);
       } else if( !strcmp("ioreg_encoding",a_node->name)){
         printf("ioreg_encoding\n");
         /**
          * recursive descent parsing.
          * Call parse_ioreg function for each child of the
          * current element.
          */
          for(cur_node = a_node->children; cur_node; cur_node = cur_node->next){
            parse_encoding(cur_node, pRegister);
          }
       } else {
          fprintf(stderr,"ERROR: one of ioreg element's children is bad <%s>\n",a_node->name);
          printf("ERROR parse_ioregister @%s:%d node_type: unexpected Element, name: %s\n",__FILE__,__LINE__,a_node->name);
          exit(1);
       }
    }
}



////////////////////////////////////////////////////////////////////////////
// parse_ioregisters  recursive descent parsing of ioregisters's children
////////////////////////////////////////////////////////////////////////////
void 
parse_ioregisters(xmlNode * a_node)
{
    T_pIORegister  pNewRegister;
    xmlNode *      cur_node;

    if(a_node->type == XML_ELEMENT_NODE) {
      // printf("parse_ioregisters @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
       if( !strcmp("ioreg_index",a_node->name)){
           printf("throwing away ioreg_index picture\n");
       } else if( !strcmp("ioreg_bigpicture",a_node->name)){
           printf("throwing away big picture\n");
       } else if( !strcmp("ioreg",a_node->name)){
          // recursive descent to process one I/O register
          /**
           * Allocate an ioreg structure and link it to device structure.
           */
          pNewRegister = (T_pIORegister)malloc(sizeof(struct _ioregister_data_struct_));
          if(pNewRegister == NULL){
            fprintf(stderr,"malloc %s %d\n",__FILE__,__LINE__);
            exit(1);
          }
          pNewRegister->pBitField = NULL;
          pNewRegister->pBitFieldTail = NULL;
          pNewRegister->Name = NULL;
          pNewRegister->Intro = NULL;
          pNewRegister->Offset = NULL;
          pNewRegister->Description = NULL;
          pNewRegister->Example = NULL;
          pNewRegister->pNext = NULL;
          if(Device.pRegisterTail==NULL){// then first time
             Device.pRegister     = pNewRegister;
             Device.pRegisterTail = pNewRegister;
          } else {
             Device.pRegisterTail->pNext = pNewRegister;
             Device.pRegisterTail = pNewRegister;
          }
          for(cur_node = a_node->children; cur_node; cur_node = cur_node->next){
            parse_ioreg(cur_node, pNewRegister);
          }
       } else {
          fprintf(stderr,"ERROR: one of ioregs element's children is bad <%s>\n",a_node->name);
          printf("ERROR parse_ioregisters @%s:%d node_type: Element, name: %s\n",__FILE__,__LINE__,a_node->name);
          exit(1);
       }
    }
}


////////////////////////////////////////////////////////////////////////////
// parse_device  recursive descent parsing of device's children
////////////////////////////////////////////////////////////////////////////
void
parse_device(xmlNode * a_node)
{
    xmlNode * cur_node;
    char *pString;
    char *pChar;
    if(a_node->type == XML_ELEMENT_NODE) {
       if( !strcmp("short_description",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          Device.ShortDescription = AddToStringStore(pString);
          printf("found short description <%s>\n", Device.ShortDescription);
          xmlFree(pString);
       } else if( !strcmp("device_name",a_node->name)){
          pString = xmlNodeGetContent(a_node);
          Device.DeviceName = AddToStringStore(pString);
          Device.LowerDeviceName = AddToStringStore(pString);
          my_lower_case_a_string(Device.LowerDeviceName);
          printf("found device_name <%s> <%s>\n",
                 Device.DeviceName, Device.LowerDeviceName);
          xmlFree(pString);
       } else if(    !strcmp("instance_base",a_node->name)) {
          pString = xmlNodeGetContent(a_node);
          printf("found instancce_base <%s> for instance %d\n",pString, Device.InstanceCount);
          if(Device.InstanceCount>3){
              fprintf(stderr,"TOO MANY INSTANCES\n");
              exit(3);
          } 
          pChar = pString;
          while(*pChar++ != ':');
          printf("parse_device: base <%s> <%s>\n",pString,pChar);
          Device.ulInstanceBase[Device.InstanceCount] = strtoul(pChar,NULL,0);
          if(Device.InstanceCount >0) Device.OnlyChild = 0;
          Device.InstanceCount++;
          xmlFree(pString);
       } else if( !strcmp("ioregisters",a_node->name)){
          /**
           * recursive descent parsing.
           * Call parse_ioregisters function for each child of the
           * current element.
           */
          printf("found io_registers now do recursive descent\n");
          for(cur_node = a_node->children; cur_node; cur_node = cur_node->next){
             parse_ioregisters(cur_node);
          }
       } else {
          printf("ERROR @%s:%d node_type: This is an Element, name: %s but I don't recognize it\n",__FILE__,__LINE__,a_node->name);
          pString = xmlNodeGetContent(a_node);
          printf("\t<%s>\n",pString);
          xmlFree(pString);
       }





    } else {
       printf("ERROR @%s:%d node_type: NOT Element, name: %s\n",__FILE__,__LINE__,a_node->name);
       printf("\t<%s>\n",xmlNodeGetContent(a_node));
    }
#if 0
    scew_list      *list = NULL;
    scew_element   *child = NULL;
    XML_Char const *contents = NULL;
    char           *device_child;


    if (element == NULL)
    {
        return;
    }

    /**
     * check for one of device element's children
     */
    device_child = (char *)scew_element_name(element);
    if( ! strcmp("short_description",device_child)){
      Device.ShortDescription = AddToStringStore((char *)scew_element_contents(element));
    } else if( ! strcmp("bus_type",device_child)){
      Device.BusType = AddToStringStore((char *)scew_element_contents(element));
    } else if( ! strcmp("device_name",device_child)){
      Device.DeviceName = AddToStringStore((char *)scew_element_contents(element));
      Device.LowerDeviceName = AddToStringStore((char *)scew_element_contents(element));
      my_lower_case_a_string(Device.LowerDeviceName);
    } else if( ! strcmp("io_device_size",device_child)){
      Device.IODeviceSize = AddToStringStore((char *)scew_element_contents(element));
    } else if( ! strcmp("ioregisters",device_child)){
      /**
       * recursive descent parsing.
       * Call parse_ioregisters function for each child of the
       * current element.
       */
      list = scew_element_children(element);
      while (list != NULL)
      {
          child = scew_list_data(list);
          parse_ioregisters(child);
          list = scew_list_next(list);
      }
    } else {
      fprintf(stderr,"ERROR: one of device element's children is bad <%s>\n",device_child);
      exit(1);
    }
#endif


}



////////////////////////////////////////////////////////////////////////////
// get device attributes
////////////////////////////////////////////////////////////////////////////
static void 
get_device_attributes(xmlNode * a_node)
{

    xmlChar *file_name;
    xmlChar *file_mod_date;

    file_name = xmlGetProp(a_node,"file_name");
    Device.FileName = AddToStringStore(file_name);
    printf("file_name: <%s> \n",Device.FileName);

    file_mod_date = xmlGetProp(a_node,"file_mod_date");
    Device.FileModDate = AddToStringStore(file_mod_date);
    printf("file_mod_date: <%s> \n",Device.FileModDate);


#if 0
    scew_list      *list = NULL;
    scew_attribute *attribute = NULL;
    char           *pAttrName; 
    char           *pAttrValue;

    if (element != NULL)
    {
        /**
         * Iterates through the element's attribute list, printing the
         * pair name-value.
         */
        list = scew_element_attributes(element);
        while (list != NULL)
        {
            attribute = scew_list_data(list);
            pAttrName   =(char *) scew_attribute_name(attribute);
            pAttrValue  =(char *) scew_attribute_value(attribute);
            if(! strcmp(pAttrName,"file_name")){
              Device.FileName = AddToStringStore(pAttrValue);
            } else {
              Device.FileModDate = AddToStringStore(pAttrValue);
            }
            list = scew_list_next(list);
        }
    }
#endif
}



////////////////////////////////////////////////////////////////////////////
//  ParseDeviceTree
////////////////////////////////////////////////////////////////////////////
int
ParseDeviceTree(char * FileName)
{
    xmlParserCtxtPtr ctxt;
    xmlNode *root_element = NULL;
  
    ctxt = xmlNewParserCtxt();
    if(ctxt == NULL){
     fprintf(stderr," unable to allocate libxml2 parser context\n");
     return;
    }
 

    Doc = xmlCtxtReadFile(ctxt,FileName,NULL,XML_PARSE_DTDVALID);
    if(Doc ==NULL) {
     fprintf(stderr," libxml2 failed to parse <%s>\n",FileName);
  
    } else{

     if(ctxt->valid == 0) {
      fprintf(stderr," libxml2 document <%s> not valid\n",FileName);
      print_element_names(0,root_element);
      //free the parser context
      xmlFreeParserCtxt(ctxt);
      return -100;
     }
    }

    root_element = xmlDocGetRootElement(Doc);
    printf("root element <%s>\n",root_element->name);
  
  
    //print_element_names(0,root_element);
  
    //free the parser context
    xmlFreeParserCtxt(ctxt);


    // first element had better be a device
    if( strcmp("device",root_element->name)){
      fprintf(stderr,"ERROR: device tag not found where expected\n");
      return -101;
    }

    // init the device structure
    Device.pRegister = NULL;
    Device.pRegisterTail = NULL;
    Device.OnlyChild = 1;
    Device.InstanceCount = 0;
    Device.ulInstanceBase[0] = 0;
    Device.ulInstanceBase[1] = 0;
    Device.ulInstanceBase[2] = 0;
    Device.ulInstanceBase[3] = 0;


    get_device_attributes(root_element);


    /**
     * recursive descent parsing.
     * Call parse_device function for each child of the
     * current element.
     */
    xmlNode *cur_node = NULL;
    for(cur_node = root_element->children; cur_node; cur_node = cur_node->next){
    if(cur_node->type == XML_ELEMENT_NODE) parse_device(cur_node);
    }

    print_device_tree(&Device, stdout);
    return 0;
}
