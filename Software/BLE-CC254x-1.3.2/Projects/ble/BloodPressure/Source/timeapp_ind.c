/**************************************************************************************************
  Filename:       timeapp_ind.c
  Revised:        $Date: 2011-06-22 20:44:48 -0700 (Wed, 22 Jun 2011) $
  Revision:       $Revision: 26428 $

  Description:    Time App indication and notification handling routines.

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

#include "string.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "timeapp.h"

/*********************************************************************
 * MACROS
 */

// Maximum category ID value
#define ALERT_CAT_ID_MAX            9

// Parse major category
#define ALERT_MAJOR_CAT(x)          ( (x) >> 3 )

// Parse subcategory
#define ALERT_SUBCAT(x)             ( (x) & 0x07 )

/*********************************************************************
 * CONSTANTS
 */


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
 * @fn      timeAppIndGattMsg
 *
 * @brief   Handle indications and notifications. 
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void timeAppIndGattMsg( gattMsgEvent_t *pMsg )
{
  uint8 i;
  
  // Look up the handle in the handle cache
  for ( i = 0; i < HDL_CACHE_LEN; i++ )
  {
    if ( pMsg->msg.handleValueNoti.handle == timeAppHdlCache[i] )
    {
      break;
    }
  }

  // Perform processing for this handle 
  switch ( i )
  {
    case HDL_CURR_TIME_CT_TIME_START:
      // Set clock to time read from time server
      timeAppClockSet( pMsg->msg.handleValueNoti.value );
      break;
      
 
     default:
      break;
  }
  
  // Send confirm for indication
  if ( pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    ATT_HandleValueCfm( pMsg->connHandle );
  }
}


/*********************************************************************
*********************************************************************/