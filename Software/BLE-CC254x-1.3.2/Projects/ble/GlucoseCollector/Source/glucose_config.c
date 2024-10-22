/**************************************************************************************************
  Filename:       glucose_config.c
  Revised:        $Date: 2011-12-16 15:46:52 -0800 (Fri, 16 Dec 2011) $
  Revision:       $Revision: 58 $

  Description:    Glucose Collector App characteristic configuration routines.

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
#include "OSAL_clock.h"
#include "OnBoard.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "glucservice.h"
#include "glucoseCollector.h"

/*********************************************************************
 * MACROS
 */

// Used to determine the end of glucoseConfigList[]
#define GLUCOSE_CONFIG_MAX      ( sizeof(glucoseConfigList) / sizeof(uint8) )

/*********************************************************************
 * CONSTANTS
 */

// Array of handle cache indexes.  This list determines the
// characteristics that are read or written during configuration.
const uint8 glucoseConfigList[] =
{
  HDL_DEVINFO_SYSTEM_ID,
  HDL_DEVINFO_MANUFACTURER_NAME,
  HDL_DEVINFO_MODEL_NUM,
  HDL_GLUCOSE_FEATURE,
  HDL_GLUCOSE_MEAS_CCCD,
  HDL_GLUCOSE_CONTEXT_CCCD,
  HDL_GLUCOSE_CTL_PNT_CCCD
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool glucCollCharHdls;

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
 * @fn      glucoseConfigNext()
 *
 * @brief   Perform the characteristic configuration read or
 *          write procedure.
 *
 * @param   state - Configuration state.
 *
 * @return  New configuration state.
 */
uint8 glucoseConfigNext( uint8 state )
{
  attReadReq_t  readReq;
  attWriteReq_t writeReq;
  bool          read;

  // Find next non-zero cached handle of interest
  while ( state < GLUCOSE_CONFIG_MAX &&
          glucoseHdlCache[glucoseConfigList[state]] == 0 )
  {
    state++;
  }

  // Return if reached end of list
  if ( state == GLUCOSE_CONFIG_MAX )
  {
    glucCollCharHdls = true;
    return GLUCOSE_CONFIG_CMPL;
  }

  // Determine what to do with characteristic
  switch ( glucoseConfigList[state] )
  {
    // Read these characteristics
    case HDL_DEVINFO_SYSTEM_ID:
    case HDL_DEVINFO_MANUFACTURER_NAME:
    case HDL_DEVINFO_MODEL_NUM:
    case HDL_GLUCOSE_FEATURE:
      read = TRUE;
      break;

    // Set notification for these characteristics
    case HDL_GLUCOSE_MEAS_CCCD:
    case HDL_GLUCOSE_CONTEXT_CCCD:
      read = FALSE;
      writeReq.len = 2;
      writeReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      writeReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
      writeReq.sig = 0;
      writeReq.cmd = 0;
      break;
      
    // Set indication for these characteristics
    case HDL_GLUCOSE_CTL_PNT_CCCD:
      read = FALSE;
      writeReq.len = 2;
      writeReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_INDICATE);
      writeReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_INDICATE);
      writeReq.sig = 0;
      writeReq.cmd = 0;
      break;
      
    default:
      break;
  }

  // Do a GATT read or write
  if ( read )
  {
    readReq.handle = glucoseHdlCache[glucoseConfigList[state]];
    GATT_ReadCharValue( glucCollConnHandle, &readReq, glucCollTaskId );
  }
  else
  {
    writeReq.handle = glucoseHdlCache[glucoseConfigList[state]];
    GATT_WriteCharValue( glucCollConnHandle, &writeReq, glucCollTaskId );
  }

  return state;
}

/*********************************************************************
 * @fn      glucoseConfigGattMsg()
   *
 * @brief   Handle GATT messages for characteristic configuration.
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New configuration state.
 */
uint8 glucoseConfigGattMsg( uint8 state, gattMsgEvent_t *pMsg )
{
  if ( (pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP) &&
       (pMsg->hdr.status == SUCCESS) )
  {
    // Process response
    switch ( glucoseConfigList[state] )
    {
      case HDL_GLUCOSE_MEAS_CCCD:
        break;
      case HDL_GLUCOSE_CONTEXT_CCCD:
        break;
      case HDL_GLUCOSE_CTL_PNT_CCCD:
        break;
      case HDL_GLUCOSE_FEATURE:
        glucoseFeatures = BUILD_UINT16(pMsg->msg.readRsp.value[0], pMsg->msg.readRsp.value[1]);
        break;
      case HDL_DEVINFO_SYSTEM_ID:
        break;
      case HDL_DEVINFO_MANUFACTURER_NAME:
        break;
      case HDL_DEVINFO_MODEL_NUM:
        break;
      default:
        break;
    }
  }

  return glucoseConfigNext( state + 1 );
}

