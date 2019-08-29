Cordio Stack & Profile
======================

Document No. ARM-EPM-115978 2.0


Legal
-----

Copyright (c) 2013-2017 ARM Ltd., all rights reserved.
ARM Ltd. confidential and proprietary.


New Features
------------

General
	* Support multiple connections
	* Support master w/ multiple slaves
	* Support simultaneous master / slave
	* Watch App to demonstrate multiple connections
	* Added Cordio Profiles Developers Guide

Changes
-------

Enhancements in r2p1-01eac

	*	WBUSW-1589: Simultaneous Master / Slave support
	*	WBUSW-1671: Multiple connection support for DM and AF
	*	WBUSW-1674: Multiple slave to master connections
	*	WBUSW-1683: Sample project Master that supports multiple slaves

Resolved defects in r2p1-01eac
	*	WBUSW-1673: Failure to copy handle and status during hci_evt processing
	*   WBUSW-1730: SMP issue with uninitialized security level actions
			
Limitations
-----------

Limitations in r2p1-01eac

General

	* WBUSW-1547: LE Set Random Address should not occur during advertising or scanning
    * WBUSW-1776: CCCD Value of WDXS got overwritten after Pairing
	
Folder Organization
-------------------
The folders in this distribution are organized as described below.

|-+ ${PRJ_ROOT}             # Root project folder
    |-+ cordio-bt4-host     # Sample Application Projects
    |   |-+ platform        #     Cordio-BT4-SDK Platform
    |   |-+ projects        #     Sample Projects
    |   |-+ sw              #     Sources
    |   |-+ projects        #     Build projects
    |   |-+ tools           #     Cordio-BT4-SDK Tools
    |-+ docs                # Documents
    |-+ projects            # Generic Projects (stacklib)
    |-+ sw                  # Common MAC files
    |   |-+ apps            #     Application Components
    |   |-+ hci             #     Upper HCI Interface
    |   |-+ profiles        #     Profiles
    |   |-+ sec             #     Security
    |   |-+ services        #     Services
    |   |-+ sw              #     Sources
    |-+ ws-core             # Wireless Software common files
        |-+ docs            #     Design and API documents
        |-+ include         #     Public interface
        |-+ projects        #     Common build files
        |-+ sw              #     Sources


Tool Dependencies
-----------------

Building and debugging projects for the Cordio BT4 Evaluation Board depend on the
latest GCC or Keil toolchains.  Please see the Cordio BT4 Customer Evaluation and
Demonstration Kit User's Guide for more information on this platform.

The GCC toolchain can be obtained from the following location:

    https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

For Windows, GNU utilities (i.e. make, grep, ls, etc.) can be obtained from:

    http://mingw.org/wiki/msys

The Keil embedded development tools can be obtained from the following location:

    http://www.keil.com/


Change History
--------------

=== Changed in r2p1-00eac ===
	
New Features
------------

General
	* Support Dynamic ATT Tables
	* Zero-Copy ATT Notifications
	* Flow Control Support for Multiple ATT Notifications
	* Buffer Pool Allocation Diagnostics
	* Thin-HCI Zero-Copy Support
	* Added WDXC and UDSC Profiles

New BLE5 features:

    * Max Throughput Application

Changes
-------

Enhancements in r2p1-00eac

	*	WBUSW-0929: Mandatory to support User Data Service for the WSP collector role
	*	WBUSW-1424: Detokenized LL trace messages
	*	WBUSW-1451: Dynamic ATT Tables
	*	WBUSW-1491: Connect DM Adv events to LL Adv enable command complete event
	*	WBUSW-1492: Zero-Copy ATT Notification/Indication
	*	WBUSW-1494: BLE5 - Create Max Throughput Apps
	*	WBUSW-1530: Add Flow Control Support for Multiple ATT Notifications
	*	WBUSW-1535: Buffer Pool Allocation – Diagnostics
	*	WBUSW-1536: Thin-HCI Zero Copy Support
	*	WBUSW-1544: ATT Signed Write Should Comply with E4243

Resolved defects in r2p1-00eac
	*	WBUSW-1338: Connection Update Request state machine problem
	*	WBUSW-1450: AppDbGetNextRecord loop goes beyond array bounds
	*	WBUSW-1484: Cordio Stack DmDevReset() for Soft Reset doesn't reset state
	*	WBUSW-1539: Stack Not Processing the Fragmented AdvData to the App Correctly
	*	WBUSW-1625: L2C_COC_CBACK_START callback event overlaps with some DM callback events

New Files
	./sw/profiles/udsc/*
	./sw/profiles/wdxc/*
	./sw/stack/att/atts_dyn.c
	./ws-core/projects/common/token2header.py
		
Limitations
-----------

Limitations in r2p1-00eac

General

	* WBUSW-1547: LE Set Random Address should not occur during advertising or scanning

BLE5.0 - Limited Testing
	
	* Synchronization with periodic advertising (AE master)


=== Changed in r2p0-10eac ===

Released: December 2016

New Features
------------

New BLE5 features:

    * LE Advertising Extensions
    * 2Mbps LE Phy (host) support
    * Host high duty cycle non-connectable advertising

New BLE4.2 features:

    * OTA support in Dats and Tag Sample Application

Changes
-------

Changes in r2p0-10eac

	* WBUSW-1210	TSE6680 support ofr LeCoC
	* WBUSW-1247	BLE5 - Host Support for LE AE Master Non-Periodic
	* WBUSW-1307	Send L2C_Rej for unidentified response PDU
	* WBUSW-1337	Connection Parameter Negotiation Issue
	* WBUSW-1367	BLE5 - Array elements should be in same order as Initiating PHYs bits
	* WBUSW-1380	BLE5 - AdvData should be allowed in extended connectable advertising type
	* WBUSW-1410	HIDApp missing events for generating and storing ECC keys
	* WBUSW-509		Core Specification Supplement (CSS) v6 changes
	* WBUSW-545		Assert if CCC set length is 0
	* WBUSW-1024	Signed Write should not be used when the link is encrypted
	* WBUSW-1076	ESR09&10, E5666 missing error code definition for Table 4.20
	* WBUSW-1104	BLE5 - Add/Handle new device type (0xFF Devices sending anonymous advertisments) in HCI
	* WBUSW-1169	Merge HID Apps
	* WBUSW-1177	Thin-HCI Sample Application Support
	* WBUSW-1198	HCI Handler ID hard coded in WsfMsgEnq
	* WBUSW-1243	HCI fails to free connection structure when connection fails
	* WBUSW-1252	MTU Size limit for indication / notification
	* WBUSW-1281	BLE5 - Host Support for LE Enhanced Privacy 1.2.1
	* WBUSW-1294	Handle OOB Request during pairing
	* WBUSW-1352	BLE5 - Create 2Mbps Sample App
	* WBUSW-1383	BLE5 - Shanghai r10 Stack Updates
	* WBUSW-1395	BLE5 - HCI VS AE modules for generic controllers and single-chip
	* WBUSW-1404	BLE5 - Extended advertising should advertise from beginning of advertising data
	* WBUSW-1413	Privacy mode should be set before advertising or scanning enabled
	* WBUSW-1414	Register UriBeacon utility service to manage CCCDs
	* WBUSW-1426	connSupervisionTimeout must be larger than (1+connSlaveLatency)*connInterval*2

New Files
		BT4 Evaluation Board
		./cordio-platform/*						
		./cordio-bt4-host/sw/board/board_rtc.c	
		OTA Support
		./cordio-bt4-host/projects/common/gcc/sources_wdxs.mk
		./cordio-bt4-host/projects/packet_ota_bin.py
		./cordio-bt4-host/projects/packet_ota_spf.py
		./cordio-bt4-host/projects/ota-boot/*
		./cordio-bt4-host/projects/common/cordio_tc-ram.sct
		./cordio-bt4-host/sw/app_param.handling
		./cordio-bt4-host/sw/ota/*
		./cordio-bt4-host/sw/board/board_rtc.c
		./ws-core/sw/wsf/common/wsf_efs.c,handling
		
Limitations
-----------

Limitations in r2p0-10eac

BLE5.0 - Limited Testing
	
	* Periodic advertising (AE slave)
	* Synchronization with periodic advertising (AE master)

=== Changed in r2p0-00bet0 ===

Released: September 2016

New BLE5 features:

    * LE Advertising Extensions (non-periodic)
    * 2Mbps LE Phy (host) support
    * Host high duty cycle non-connectable advertising

New BLE4.2 features:

    * Sample applications available for Cordio BT4 SDK
    * Cycling Power Profile and Service
    * Cycling Speed and Cadence Profile and Service
    * Running Speed and Cadence Profile and Service
    * Pulse Oximeter Profile and Service
    * Internet Protocol Support Service
    * Scan Parameters Profile (ScPP) and Scan Parameters Service (ScPS)
	
Bug Fixes and Enhancements

    * WBUSW-708     BLE5 - Host Support for 2 Mbps LE PHY
    * WBUSW-751     Move stack-specific WSF files to stack directories
    * WBUSW-907     Stack should not send another ATT request when the previous ATT timeout
    * WBUSW-964     GLS does not request Security on Glucose Feature, but the profile mandates
    * WBUSW-967     Dats stops working if sending AppBtnUiTest(2) when connected
    * WBUSW-980     Unsafe Pointer Usage with Callback Events
    * WBUSW-981     Add Scan Parameters Profile (ScPP) and Scan Parameters Service (ScPS)
    * WBUSW-986     LESC passkey responder fails with some devices
    * WBUSW-989     Cycling Power Profile and Service
    * WBUSW-1004    LESC OOB doesn't skip the confirm check when the OOB data present flag is FALSE
    * WBUSW-1008    Add New Profiles and Services
    * WBUSW-1042    BLE5 - Host Support for LE Advertising Extensions
    * WBUSW-1101    Signed Writes need port to new CMAC
    * WBUSW-1107    Remove Stack Generic Projects
    * WBUSW-1108    BLE5 – Host Support for High Duty Cycle Non-Connectable Advertising
    * WBUSW-1115    Update documentation to ARM format
    * WBUSW-1121    Add Glucose service to MEDS app
    * WBUSW-1170    ScPP Callback Enhancement
    * WBUSW-1171    BLE5 - Add Thin-HCI Support for Advertising Extensions
    * WBUSW-1172    BLE5 - Add Thin-HCI Support for LE 2Mbps
    * WBUSW-1173    HCI_EVT not properly handling ADV_REPORT_IND
    * WBUSW-1183    Signed Write Not Working with New CMAC

Known Issues

    * WBUSW-1024    Signed Write should not be used when the link is encrypted
    * WBUSW-1193    MBED flash programming fails for large binaries


=== Changed in exactle_01.03.02_r7043 ===

Released: May 9, 2016

New Features:

    * Profile qualification listing update
    * IMPORTANT NOTE:  File hci_defs.h has been moved to a new directory /sw/include.  The include
      path in any of your makefiles or project files will need to be updated.

Bug Fixes and Enhancements:

    #900    Synchronize hci_defs.h with LL
    #921    Clean warnings generated with 'cast-align' & 'switch-default' options
    #948    Add a configuration flag in Medc to indicate whether or not to disconnect upon ATT timeout
    #939    Read Glucose Feature should not required security
    #940    gluc: Glucose sensor application button enhancement
    #938    GLS should return invalid operator (0x03)
    #932    WSS reserved bit is set as default for Weight Scale Feature Characteristic
    #926    Keyboard, Mouse and Remote enfores security to read HID service declaration
    #482    Allow DIS values to be application-configurable
    #797    Replace 'wicentric app' name in stack apps
    #908    LL connection parameter events do not match "thin HCI" fields

Known Issues:

    #907    Stack should not send another ATT request when the previous ATT timeout
    #964    GLS does not request Security on Glucose Feature, but the profile mandates
    #967    'Dats' sample app stops working if sending AppUiBtnTest(2) when connected


=== Changed in exactle_01.03.01_r6636 ===

Release: April 5, 2016

New Features:

    * Host Support for LE Remote Connection Parameter Request
    * LE Ping commands/event
    * 4.2 qualification listing update

Bug Fixes and Enhancements:

    #783    LESC HCI commands/events are little-endian
    #799    Pass command complete for resolving list commands up to app
    #844    LESC Clear RA in OOB should happen based on device's own smp cfg
    #765    DHKey Check Fails when Re-Pairing
    #860    App Slave should check for peer’s LL Address Resolution support
    #863    Pass 'Clear Resolving List' Command to DM Privacy
    #858    Add CSRK and Sign Counter Support to App DB

Known Issues:

    #908    LL connection parameter events do not match "thin HCI" fields


=== Changed in exactle_01.03.00_r5781 ===

Released: February 12, 2016

New Features:

    * Support for Bluetooth 4.2 extended length packets.
    * Support for Bluetooth 4.2 enhanced privacy.

Bug Fixes and Enhancements:

    #328    Convert bstream macros to functions to reduce code size
    #341    Mouse Only HID Profile
    #352    update qualification test ATT services
    #395    Revert ATT permission change related to LESC
    #411    DM 4.2 spec updates.
    #469    Dereference of NULL pointer after check in WSF function
    #524    Improve comments in wsf_sec.h
    #536    Host packet size configuration
    #550    L2CAP COC peer MPS not set correctly
    #551    LESC Passkey selected instead of Numeric Comparison
    #552    Privacy 1.1 Passkey Entry selected instead of Passkey Display
    #554    Numeric Verification Value Not Correct
    #559    HCI Support for Enhanced Privacy 1.2
    #574    Host Support for Enhanced Privacy 1.2
    #578    HCI Enhancements
    #617    Encryption setup using SC LTK fails
    #638    Host support for LE Data Packet Length Extensions
    #749    DM Support for LE Set Data Length Command
    #757    Re-pairing fails after reconnection

Known Issues:

    #765    DHKey Check Fails when Re-Pairing with LE secure connections