/**************************************************************************************************
  Filename:       hci_ext_app.h
  Revised:        $Date: 2012-08-28 10:08:06 -0700 (Tue, 28 Aug 2012) $
  Revision:       $Revision: 31399 $

  Description:    HCI Extensions Application.

  Copyright 2009 - 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef HCI_EXT_APP_H
#define HCI_EXT_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
/*** HCI Extension Commands ***/

// The 10-bit OCF (Opcode Command Field) of the HCI Opcode is further
// devided into two subsections:
// - Subgroup (3 bits):
//   - 0 (LL)
//   - 1 (L2CAP)
//   - 2 (ATT)
//   - 3 (GATT)
//   - 4 (GAP)
//   - 5 (UTIL)
//   - 6 (Reserved)
//   - 7 (User Profile)
// - Command (7 bits) or Profile (7 bits) if Subgroup value is set to
//   User Profile (i.e., all ones) in which case, another octet is
//   required to represent user profile commands.
//
#define HCI_EXT_LL_SUBGRP                     0x00
#define HCI_EXT_L2CAP_SUBGRP                  0x01
#define HCI_EXT_ATT_SUBGRP                    0x02
#define HCI_EXT_GATT_SUBGRP                   0x03
#define HCI_EXT_GAP_SUBGRP                    0x04
#define HCI_EXT_UTIL_SUBGRP                   0x05
#define HCI_EXT_PROFILE_SUBGRP                0x07

#define HCI_EXT_UTIL_RESET                    0x00
#define HCI_EXT_UTIL_NV_READ                  0x01
#define HCI_EXT_UTIL_NV_WRITE                 0x02
#define HCI_EXT_UTIL_FORCE_BOOT               0x03

// GAP Initialization and Configuration
#define HCI_EXT_GAP_DEVICE_INIT               0x00
#define HCI_EXT_GAP_CONFIG_DEVICE_ADDR        0x03

// GAP Device Discovery
#define HCI_EXT_GAP_DEVICE_DISC_REQ           0x04
#define HCI_EXT_GAP_DEVICE_DISC_CANCEL        0x05
#define HCI_EXT_GAP_MAKE_DISCOVERABLE         0x06
#define HCI_EXT_GAP_UPDATE_ADV_DATA           0x07
#define HCI_EXT_GAP_END_DISC                  0x08

// GAP Link Establishment
#define HCI_EXT_GAP_EST_LINK_REQ              0x09
#define HCI_EXT_GAP_TERMINATE_LINK            0x0A
#define HCI_EXT_GAP_AUTHENTICATE              0x0B
#define HCI_EXT_GAP_PASSKEY_UPDATE            0x0C
#define HCI_EXT_GAP_SLAVE_SECURITY_REQ_UPDATE 0x0D
#define HCI_EXT_GAP_SIGNABLE                  0x0E
#define HCI_EXT_GAP_BOND                      0x0F
#define HCI_EXT_GAP_TERMINATE_AUTH            0x10
#define HCI_EXT_GAP_UPDATE_LINK_PARAM_REQ     0x11

// GAP Parameters
#define HCI_EXT_GAP_SET_PARAM                 0x30
#define HCI_EXT_GAP_GET_PARAM                 0x31
#define HCI_EXT_GAP_RESOLVE_PRIVATE_ADDR      0x32
#define HCI_EXT_GAP_SET_ADV_TOKEN             0x33
#define HCI_EXT_GAP_REMOVE_ADV_TOKEN          0x34
#define HCI_EXT_GAP_UPDATE_ADV_TOKENS         0x35
#define HCI_EXT_GAP_BOND_SET_PARAM            0x36
#define HCI_EXT_GAP_BOND_GET_PARAM            0x37
#define HCI_EXT_GAP_BOND_SERVICE_CHANGE       0x38

// GATT Sub-Procedure Commands
#define GATT_FIND_INCLUDED_SERVICES           0x30
#define GATT_DISC_ALL_CHARS                   0x32
#define GATT_READ_USING_CHAR_UUID             0x34
#define GATT_WRITE_NO_RSP                     0x36
#define GATT_SIGNED_WRITE_NO_RSP              0x38
#define GATT_RELIABLE_WRITES                  0x3a
#define GATT_READ_CHAR_DESC                   0x3c
#define GATT_READ_LONG_CHAR_DESC              0x3e
#define GATT_WRITE_CHAR_DESC                  0x40
#define GATT_WRITE_LONG_CHAR_DESC             0x42

// GATT HCI Extension messages (0x7C - 0x7F)
#define HCI_EXT_GATT_ADD_SERVICE              ( GATT_BASE_METHOD | 0x3C ) // 0x7C
#define HCI_EXT_GATT_DEL_SERVICE              ( GATT_BASE_METHOD | 0x3D ) // 0x7D
#define HCI_EXT_GATT_ADD_ATTRIBUTE            ( GATT_BASE_METHOD | 0x3E ) // 0x7E

/*** HCI Extension Events ***/

// HCI extension events must start from 0x0400. The upper 6 bits of all
// zeros is reserved for the HCI extension embedded commands. The rest of
// the event bits should follow the HCI extension command format (i.e.,
// 3-bit Subgroup + 7-bit Event).
//
#define HCI_EXT_BASE_EVENT                    ( 0x0001 << 10 )  // 0x0400

#define HCI_EXT_LL_EVENT                      ( HCI_EXT_BASE_EVENT | (HCI_EXT_LL_SUBGRP << 7) )    // 0x0400
#define HCI_EXT_L2CAP_EVENT                   ( HCI_EXT_BASE_EVENT | (HCI_EXT_L2CAP_SUBGRP << 7) ) // 0x0480
#define HCI_EXT_ATT_EVENT                     ( HCI_EXT_BASE_EVENT | (HCI_EXT_ATT_SUBGRP << 7) )   // 0x0500
#define HCI_EXT_GATT_EVENT                    ( HCI_EXT_BASE_EVENT | (HCI_EXT_GATT_SUBGRP << 7) )  // 0x0580
#define HCI_EXT_GAP_EVENT                     ( HCI_EXT_BASE_EVENT | (HCI_EXT_GAP_SUBGRP << 7) )   // 0x0600

// GAP Events
#define HCI_EXT_GAP_DEVICE_INIT_DONE_EVENT          ( HCI_EXT_GAP_EVENT | 0x00 )
#define HCI_EXT_GAP_DEVICE_DISCOVERY_EVENT          ( HCI_EXT_GAP_EVENT | 0x01 )
#define HCI_EXT_GAP_ADV_DATA_UPDATE_DONE_EVENT      ( HCI_EXT_GAP_EVENT | 0x02 )
#define HCI_EXT_GAP_MAKE_DISCOVERABLE_DONE_EVENT    ( HCI_EXT_GAP_EVENT | 0x03 )
#define HCI_EXT_GAP_END_DISCOVERABLE_DONE_EVENT     ( HCI_EXT_GAP_EVENT | 0x04 )
#define HCI_EXT_GAP_LINK_ESTABLISHED_EVENT          ( HCI_EXT_GAP_EVENT | 0x05 )
#define HCI_EXT_GAP_LINK_TERMINATED_EVENT           ( HCI_EXT_GAP_EVENT | 0x06 )
#define HCI_EXT_GAP_LINK_PARAM_UPDATE_EVENT         ( HCI_EXT_GAP_EVENT | 0x07 )
#define HCI_EXT_GAP_RANDOM_ADDR_CHANGED_EVENT       ( HCI_EXT_GAP_EVENT | 0x08 )
#define HCI_EXT_GAP_SIGNATURE_UPDATED_EVENT         ( HCI_EXT_GAP_EVENT | 0x09 )
#define HCI_EXT_GAP_AUTH_COMPLETE_EVENT             ( HCI_EXT_GAP_EVENT | 0x0A )
#define HCI_EXT_GAP_PASSKEY_NEEDED_EVENT            ( HCI_EXT_GAP_EVENT | 0x0B )
#define HCI_EXT_GAP_SLAVE_REQUESTED_SECURITY_EVENT  ( HCI_EXT_GAP_EVENT | 0x0C )
#define HCI_EXT_GAP_DEVICE_INFO_EVENT               ( HCI_EXT_GAP_EVENT | 0x0D )
#define HCI_EXT_GAP_BOND_COMPLETE_EVENT             ( HCI_EXT_GAP_EVENT | 0x0E )
#define HCI_EXT_GAP_PAIRING_REQ_EVENT               ( HCI_EXT_GAP_EVENT | 0x0F )

#define HCI_EXT_GAP_CMD_STATUS_EVENT                ( HCI_EXT_GAP_EVENT | 0x7F )

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the task
 */
extern void HCI_EXT_App_Init( uint8 task_id );

/*
 * Task Event Processor for the task
 */
extern uint16 HCI_EXT_App_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HCI_EXT_APP_H */
