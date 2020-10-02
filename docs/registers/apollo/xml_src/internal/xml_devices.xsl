<?xml version="1.0"?>
<!--
=============================================================================
==
== Filename xml_devices.xsl
==
=============================================================================
==  COPYRIGHT 2003 A6LABS
=============================================================================
-->

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

 <xsl:template match="device">
 <HTML>
   <HEAD>
	   <TITLE>IOXML FILE: <xsl:value-of select="short_description"/></TITLE>
   </HEAD>
   <BODY>
      <H1 align="center">On-Line Documentation</H1>
      <FONT  size="+2">
	      <B><P align="center"><xsl:value-of select="short_description"/></P></B>
	      <B><P align="center">BUS TYPE: <xsl:value-of select="bus_type"/></P></B>
      </FONT>
      <HR/>
      <FONT face="Helvetica" size="-1">
         <P align="center">
           <B> Put banner announcement here. </B>
	 </P>
      </FONT>
      <HR/>
      <xsl:apply-templates select="ioregisters"/>
      <BR/><BR/>
      <HR/>
      <FONT size="-2">
      <P id="revision">XML Source File Version Information</P>
        <P><B>File Name:</B> <xsl:value-of select="@file_name"/></P>
        <P><B>File Modification Date:</B> <xsl:value-of select="@file_mod_date"/></P>
      </FONT>
   </BODY>
 </HTML>
 </xsl:template>

 <xsl:template match="ioregisters">
    <xsl:apply-templates select="ioreg_index"/>
    <xsl:apply-templates select="ioreg_bigpicture"/>
    <xsl:apply-templates select="ioreg"/>
 </xsl:template>

 <xsl:template match="ioreg_index">
	 <H2 id="index"><xsl:value-of select="/device/device_name"/> Register Index</H2>
	 <P>
		 <B> Device I/O size = </B> <TT> <xsl:value-of select="/device/io_device_size"/></TT>
	 </P>
	 <xsl:apply-templates select="/device/ioregisters/ioreg" mode="index"/>
 </xsl:template>

 <xsl:template match="ioreg" mode="index">
	 <B><TT><xsl:value-of select="ioreg_offset"/>: </TT></B>
	 <A id="{ioreg_name}index" href="#{ioreg_name}">
		 <B>IO_<xsl:value-of select="/device/device_name"/>_<xsl:value-of select="ioreg_name"/></B>
		 -
		 <xsl:value-of select="ioreg_intro"/>
	 </A>
	 <BR/>
 </xsl:template>


   <xsl:template match="ioreg_bigpicture">
      <BR/>
      <HR/>
      <H2 id="bigpicture"><xsl:value-of select="/device/device_name"/> Register Format Summary</H2>
      <TABLE border="1" cellpadding="1" cellspacing="0">
         <TR align="top" bgcolor="#888888">
            <TD><SMALL><B>Register</B></SMALL></TD>
            <TD><SMALL><B>31</B></SMALL></TD>
            <TD><SMALL><B>30</B></SMALL></TD>
            <TD><SMALL><B>29</B></SMALL></TD>
            <TD><SMALL><B>28</B></SMALL></TD>
            <TD><SMALL><B>27</B></SMALL></TD>
            <TD><SMALL><B>26</B></SMALL></TD>
            <TD><SMALL><B>25</B></SMALL></TD>
            <TD><SMALL><B>24</B></SMALL></TD>
            <TD><SMALL><B>23</B></SMALL></TD>
            <TD><SMALL><B>22</B></SMALL></TD>
            <TD><SMALL><B>21</B></SMALL></TD>
            <TD><SMALL><B>20</B></SMALL></TD>
            <TD><SMALL><B>19</B></SMALL></TD>
            <TD><SMALL><B>18</B></SMALL></TD>
            <TD><SMALL><B>17</B></SMALL></TD>
            <TD><SMALL><B>16</B></SMALL></TD>
            <TD><SMALL><B>15</B></SMALL></TD>
            <TD><SMALL><B>14</B></SMALL></TD>
            <TD><SMALL><B>13</B></SMALL></TD>
            <TD><SMALL><B>12</B></SMALL></TD>
            <TD><SMALL><B>11</B></SMALL></TD>
            <TD><SMALL><B>10</B></SMALL></TD>
            <TD><SMALL><B>9</B></SMALL></TD>
            <TD><SMALL><B>8</B></SMALL></TD>
            <TD><SMALL><B>7</B></SMALL></TD>
            <TD><SMALL><B>6</B></SMALL></TD>
            <TD><SMALL><B>5</B></SMALL></TD>
            <TD><SMALL><B>4</B></SMALL></TD>
            <TD><SMALL><B>3</B></SMALL></TD>
            <TD><SMALL><B>2</B></SMALL></TD>
            <TD><SMALL><B>1</B></SMALL></TD>
            <TD><SMALL><B>0</B></SMALL></TD>
         </TR>

         <xsl:apply-templates select="/device/ioregisters/ioreg" mode="bigpicture"/>
      </TABLE>
      <BR/>
   </xsl:template>

   <xsl:template match="ioreg" mode="bigpicture">
      <TR align="top">
         <TH align="left" bgcolor="#888888">
            <A href="#{ioreg_name}"> 
      IO_<xsl:value-of select="/device/device_name"/>_<xsl:value-of select="ioreg_name"/>
            </A>
            <BR/>
            <TT><xsl:value-of select="ioreg_offset"/></TT>
            </TH>
         <xsl:apply-templates select="ioreg_encoding/ioreg_bitfield"/>
      </TR>
   </xsl:template>



   <xsl:template match="ioreg_bitfield">
      <xsl:choose>
         <xsl:when test="@reserved='reserved'">
            <TD bgcolor="#c0c0c0" colspan="{@width}">
               <xsl:apply-templates select="@name"/><BR/>
               <TT><xsl:apply-templates select="@reset_value"/></TT>
            </TD>
         </xsl:when>
         <xsl:when test="@readwrite='RO'">
            <TD bgcolor="#e0e0e0" colspan="{@width}">
               <xsl:apply-templates select="@name"/><BR/>
               <TT><xsl:apply-templates select="@reset_value"/></TT>
            </TD>
         </xsl:when>
         <xsl:otherwise>
            <TD bgcolor="#ffffff" colspan="{@width}">
               <xsl:apply-templates select="@name"/><BR/>
               <TT><xsl:apply-templates select="@reset_value"/></TT>
            </TD>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <xsl:template match="ioreg">
      <BR/>
      <HR/>
      <H2 id="{ioreg_name}">
         <A href="#index">
            IO_<xsl:value-of select="/device/device_name"/>_<xsl:value-of select="ioreg_name"/>
         </A>
      </H2>
      <P>
        <FONT  size="+1"> 
          <B><xsl:value-of select="ioreg_intro"/></B>
        </FONT>
      </P>
      <P>
         <TABLE>
            <TR>
               <TD><B>REGISTER OFFSET:</B></TD>
               <TD><TT><xsl:value-of select="ioreg_offset"/></TT></TD>
            </TR>
         </TABLE>
         <BR/>
         <TABLE border="1" cellpadding="1" cellspacing="0">
            <TR align="top" bgcolor="#888888">
               <TD><SMALL><B>31</B></SMALL></TD>
               <TD><SMALL><B>30</B></SMALL></TD>
               <TD><SMALL><B>29</B></SMALL></TD>
               <TD><SMALL><B>28</B></SMALL></TD>
               <TD><SMALL><B>27</B></SMALL></TD>
               <TD><SMALL><B>26</B></SMALL></TD>
               <TD><SMALL><B>25</B></SMALL></TD>
               <TD><SMALL><B>24</B></SMALL></TD>
               <TD><SMALL><B>23</B></SMALL></TD>
               <TD><SMALL><B>22</B></SMALL></TD>
               <TD><SMALL><B>21</B></SMALL></TD>
               <TD><SMALL><B>20</B></SMALL></TD>
               <TD><SMALL><B>19</B></SMALL></TD>
               <TD><SMALL><B>18</B></SMALL></TD>
               <TD><SMALL><B>17</B></SMALL></TD>
               <TD><SMALL><B>16</B></SMALL></TD>
               <TD><SMALL><B>15</B></SMALL></TD>
               <TD><SMALL><B>14</B></SMALL></TD>
               <TD><SMALL><B>13</B></SMALL></TD>
               <TD><SMALL><B>12</B></SMALL></TD>
               <TD><SMALL><B>11</B></SMALL></TD>
               <TD><SMALL><B>10</B></SMALL></TD>
               <TD><SMALL><B>9</B></SMALL></TD>
               <TD><SMALL><B>8</B></SMALL></TD>
               <TD><SMALL><B>7</B></SMALL></TD>
               <TD><SMALL><B>6</B></SMALL></TD>
               <TD><SMALL><B>5</B></SMALL></TD>
               <TD><SMALL><B>4</B></SMALL></TD>
               <TD><SMALL><B>3</B></SMALL></TD>
               <TD><SMALL><B>2</B></SMALL></TD>
               <TD><SMALL><B>1</B></SMALL></TD>
               <TD><SMALL><B>0</B></SMALL></TD>
            </TR>
            <xsl:apply-templates select="ioreg_encoding"/>
         </TABLE>
      </P>
      <TABLE border="1" cellpadding="3" cellspacing="0">
         <TR align="left" bgcolor="#888888">
            <TH><U><SMALL>Bits</SMALL></U></TH>
            <TH><U><SMALL>Name</SMALL></U></TH>
            <TH><U><SMALL>RW</SMALL></U></TH>
            <TH><U><SMALL>Flags</SMALL></U></TH>
            <TH><U><SMALL>Description</SMALL></U></TH>
         </TR>
         <xsl:apply-templates select="ioreg_encoding" mode="bitfield"/>
      </TABLE>
      <P>
         <B>Description:</B><BR/>
         <xsl:value-of select="ioreg_description"/>
      </P>
        <xsl:choose>
           <xsl:when test="@skip='skip'">
               <P>
                  <B>SKIPPED IN GRD FILE</B>
               </P>
           </xsl:when>
           <xsl:otherwise>
           </xsl:otherwise>
        </xsl:choose>
      <P>
         <B>Examples:</B>
         <TABLE bgcolor="#e0e0e0" border="1" cellpadding="5" cellspacing="0">
            <TR><TD><PRE><xsl:apply-templates select="ioreg_example"/></PRE></TD></TR>
         </TABLE>
      </P>
   </xsl:template>

   <xsl:template match="ioreg_encoding" mode="bitfield">
	   <xsl:apply-templates select="ioreg_bitfield" mode="bitfield"/>
   </xsl:template>

   <xsl:template match="ioreg_encoding">
	   <TR align="top"><xsl:apply-templates select="ioreg_bitfield"/></TR>
   </xsl:template>

   <xsl:template match="ioreg_bitfield" mode="bitfield">
     <TR align="left">
	     <TD nowrap="1"><xsl:value-of select="@lsb + @width - 1"/>:<xsl:apply-templates select="@lsb"/></TD>
        <TD nowrap="1"><xsl:apply-templates select="@name"/></TD>
        <TD nowrap="1"><xsl:apply-templates select="@readwrite"/></TD>
	<TD>
            <xsl:apply-templates select="@reserved"/>
            <xsl:choose>
               <xsl:when test="@interrupt='IRQ'">
                 <xsl:apply-templates select="@interrupt"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@empty='EMPTY'">
                 <xsl:apply-templates select="@empty"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@softreset='SFTRST'">
		       <xsl:apply-templates select="@softreset"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@backdoor='BACKDOOR'">
		       <xsl:apply-templates select="@backdoor"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@bd_set='BD_SET'">
		       <xsl:apply-templates select="@bd_set"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@bd_reset='BD_RESET'">
		       <xsl:apply-templates select="@bd_reset"/><BR/>
               </xsl:when>
            </xsl:choose>
            <xsl:choose>
               <xsl:when test="@do_not_autotest='DO_NOT_AUTOTEST'">
                  <xsl:apply-templates select="@do_not_autotest"/><BR/>
               </xsl:when>
            </xsl:choose>
        </TD>
	<TD>
           <xsl:value-of select="ioreg_bf_description"/>
	   <TABLE>
               <xsl:apply-templates select="ioreg_bf_enum" mode="row"/>
	   </TABLE>
        </TD>
     </TR>
   </xsl:template>
   
   <xsl:template match="ioreg_bf_enum" mode="row">
      <TR>
        <TD><SMALL><TT><xsl:value-of select="@name"/></TT></SMALL></TD>
	<TD><SMALL><TT>=</TT></SMALL></TD> 
	<TD><SMALL><TT><xsl:value-of select="@value"/></TT></SMALL></TD>
	<TD width="5"><SMALL><TT>-</TT></SMALL></TD> 
        <TD><SMALL><TT><xsl:value-of select="."/></TT></SMALL></TD>
      </TR>
   </xsl:template>

</xsl:stylesheet>
