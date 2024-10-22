/**************************************************************************************************
  Filename:       timeapp_config.c
  Revised:        $Date: 2012-01-20 14:33:21 -0800 (Fri, 20 Jan 2012) $
  Revision:       $Revision: 63 $

  Description:    Time App characteristic configuration routines.

  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "timeapp.h"

/*********************************************************************
 * MACROS
 */

// Used to determine the end of timeAppConfigList[]
#define TIMEAPP_CONFIG_MAX      ( sizeof(timeAppConfigList) / sizeof(uint8) )

/*********************************************************************
 * CONSTANTS
 */

// Array of handle cache indexes.  This list determines the
// characteristics that are read or written during configuration.
const uint8 timeAppConfigList[] =
{
  HDL_CURR_TIME_CT_TIME_START,            // Current time
  HDL_CURR_TIME_LOC_INFO,                 // Local time info
  HDL_CURR_TIME_REF_INFO,                 // Reference time info
  HDL_DST_CHG_TIME_DST,                   // Time with DST
  HDL_NWA_NWA_START,                      // NwA
  HDL_BATT_LEVEL_START,                   // Battery level
  HDL_PAS_RINGER_START,                   // Ringer setting start handle
  HDL_REF_TIME_UPD_STATE,                 // Time update state
  HDL_CURR_TIME_CT_TIME_CCCD,             // Current time CCCD
  HDL_NWA_NWA_CCCD,                       // NwA CCCD
  HDL_ALERT_NTF_UNREAD_CCCD,              // Unread alert status CCCD
  HDL_ALERT_NTF_NEW_CCCD,                 // New alert CCCD
  HDL_BATT_LEVEL_CCCD,                    // Battery level CCCD
  HDL_PAS_ALERT_CCCD,                     // Alert status CCCD
  HDL_PAS_RINGER_CCCD,                    // Ringer setting CCCD
  
  // these characteristics are configured at connection setup
  HDL_PAS_ALERT_START,                    // Alert status start handle
  HDL_ALERT_NTF_NEW_CAT,                  // Supported New Alert Category
  HDL_ALERT_NTF_UNREAD_CAT,               // Supported Unread Alert Category
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_CTRL                      // Alert notification control point
};

// start index of alert notification control point in config list
#define TIMEAPP_ALERT_NTF_CTRL_START    18

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      timeAppConfigNext()
 *
 * @brief   Perform the characteristic configuration read or
 *          write procedure.
 *
 * @param   state - Configuration state.
 *
 * @return  New configuration state.
 */
uint8 timeAppConfigNext( uint8 state )
{
  attReadReq_t  readReq;
  attWriteReq_t writeReq;
  bool          read;
  static uint8  alertNtfCtrlCmd;
  
  // Find next non-zero cached handle of interest
  while ( state < TIMEAPP_CONFIG_MAX &&
          timeAppHdlCache[timeAppConfigList[state]] == 0 )
  {
    state++;
  }

  // Return if reached end of list
  if ( state == TIMEAPP_CONFIG_MAX )
  {
    return TIMEAPP_CONFIG_CMPL;
  }

  // Determine what to do with characteristic
  switch ( timeAppConfigList[state] )
  {
    // Read these characteristics
    case HDL_CURR_TIME_LOC_INFO:
    case HDL_CURR_TIME_REF_INFO:
    case HDL_DST_CHG_TIME_DST:
    case HDL_NWA_NWA_START:
    case HDL_CURR_TIME_CT_TIME_START:
    case HDL_ALERT_NTF_NEW_CAT:
    case HDL_ALERT_NTF_UNREAD_CAT:
    case HDL_PAS_ALERT_START:
    case HDL_PAS_RINGER_START:  
    case HDL_REF_TIME_UPD_STATE:
      read = TRUE;
      break;

    // Set notification for these characteristics
    case HDL_CURR_TIME_CT_TIME_CCCD:
    case HDL_ALERT_NTF_UNREAD_CCCD:
    case HDL_ALERT_NTF_NEW_CCCD:
    case HDL_BATT_LEVEL_CCCD:
    case HDL_PAS_ALERT_CCCD:
    case HDL_PAS_RINGER_CCCD:
      read = FALSE;
      writeReq.len = 2;
      writeReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      writeReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
      writeReq.sig = 0;
      writeReq.cmd = 0;
      break;

    // Set indication for these characteristics
    case HDL_NWA_NWA_CCCD:
      read = FALSE;
      writeReq.len = 2;
      writeReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_INDICATE);
      writeReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_INDICATE);
      writeReq.sig = 0;
      writeReq.cmd = 0;
      break;

    // Alert notification control point
    case HDL_ALERT_NTF_CTRL:
      // initialize control point command
      if (state == TIMEAPP_ALERT_NTF_CTRL_START)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_ENABLE_NEW;
      }
      
      read = FALSE;
      writeReq.len = 2;
      writeReq.value[0] = alertNtfCtrlCmd;
      writeReq.value[1] = ALERT_NOTIF_CAT_ALL;
      writeReq.sig = 0;
      writeReq.cmd = 0;
      
      // set next command to send
      if (alertNtfCtrlCmd == ALERT_NOTIF_ENABLE_NEW)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_NOTIFY_NEW;
      }
      else if (alertNtfCtrlCmd == ALERT_NOTIF_NOTIFY_NEW)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_ENABLE_UNREAD;
      }
      else if (alertNtfCtrlCmd == ALERT_NOTIF_ENABLE_UNREAD)
      {
        alertNtfCtrlCmd = ALERT_NOTIF_NOTIFY_UNREAD;
      }
      break;
      
    default:
      break;
  }

  // Do a GATT read or write
  if ( read )
  {
    readReq.handle = timeAppHdlCache[timeAppConfigList[state]];
    GATT_ReadCharValue( timeAppConnHandle, &readReq, timeAppTaskId );
  }
  else
  {
    writeReq.handle = timeAppHdlCache[timeAppConfigList[state]];
    GATT_WriteCharValue( timeAppConnHandle, &writeReq, timeAppTaskId );
  }

  return state;
}

/*********************************************************************
 * @fn      timeAppConfigGattMsg()
   *
 * @brief   Handle GATT messages for characteristic configuration.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New configuration state.
 */
uint8 timeAppConfigGattMsg( uint8 state, gattMsgEvent_t *pMsg )
{
  if ( state > TIMEAPP_CONFIG_MAX )
  {
    return TIMEAPP_CONFIG_CMPL;
  }
  
  if ( (pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP) &&
       (pMsg->hdr.status == SUCCESS) )
  {
    // Process response
    switch ( timeAppConfigList[state] )
    {
      case HDL_CURR_TIME_CT_TIME_START:
        // Set clock to time read from time server
        timeAppClockSet( pMsg->msg.readRsp.value );
        break;

      case HDL_CURR_TIME_LOC_INFO:
        break;

      case HDL_CURR_TIME_REF_INFO:
        break;

      case HDL_DST_CHG_TIME_DST:
        break;

      case HDL_NWA_NWA_START:
        // Display network availability state
        if ( pMsg->msg.readRsp.value[0] == 1 )
        {
          LCD_WRITE_STRING( "Network: Yes", HAL_LCD_LINE_1 );
        }
        else
        {
          LCD_WRITE_STRING( "Network: None", HAL_LCD_LINE_1 );
        }    
        break;

      case HDL_BATT_LEVEL_START:
        // Display battery level
        LCD_WRITE_STRING_VALUE( "Battery%", pMsg->msg.readRsp.value[0], 10, HAL_LCD_LINE_2 );
        break;

      case HDL_CURR_TIME_CT_TIME_CCCD:
        break;

      case HDL_ALERT_NTF_NEW_CAT:
        break;

      case HDL_ALERT_NTF_UNREAD_CAT:
        break;
        
      case HDL_ALERT_NTF_UNREAD_CCCD:
        break;

      case HDL_ALERT_NTF_NEW_CCCD:
        break;

      case HDL_NWA_NWA_CCCD:
        break;

      case HDL_PAS_ALERT_START:
        // Display phone alert status
        LCD_WRITE_STRING_VALUE( "Phone Alert:", pMsg->msg.readRsp.value[0], 16, HAL_LCD_LINE_1 );
        break;

      case HDL_PAS_RINGER_START:
        // Display ringer state
        if ( pMsg->msg.readRsp.value[0] == 0 )
        {
          LCD_WRITE_STRING( "Ringer Off", HAL_LCD_LINE_2 );
        }
        else
        {
          LCD_WRITE_STRING( "Ringer On", HAL_LCD_LINE_2 );
        }    
        break;

      default:
        break;
    }
  }

  return timeAppConfigNext( state + 1 );
}

