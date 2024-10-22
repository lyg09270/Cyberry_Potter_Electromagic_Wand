/**************************************************************************************************
  Filename:       glucose_ctlpnt.c
  Revised:        $Date: 2011-10-13 22:20:07 -0700 (Thu, 13 Oct 2011) $
  Revision:       $Revision: 42 $

  Description:    Glucose Collector indication and notification handling routines.

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
#include "hal_lcd.h"
#include "gatt.h"
#include "glucservice.h"
#include "glucoseCollector.h"

/*********************************************************************
 * MACROS
 */

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
 * @fn      glucoseCtlPntWrite
 *
 * @brief   Write Control Point Requests
 *
 * @param   opcode - control point opcode
 *          oper - control point operator
 *
 * @return  status of write
 */

uint8 glucoseCtlPntWrite(uint8 opcode, uint8 oper)
{
  attWriteReq_t writeReq;
  
  writeReq.value[0] = opcode;
  writeReq.value[1] = oper;
  
  writeReq.len = 2;  
  writeReq.sig = 0;
  writeReq.cmd = 0;
  
  writeReq.handle = glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START];
  return GATT_WriteCharValue( glucCollConnHandle, &writeReq, glucCollTaskId );
}

/*********************************************************************
 * @fn      glucoseCtlPntWriteFilter
 *
 * @brief   Write Control Point Filter Requests
 *
 * @param   opcode - control point opcode
 * @param   oper - control point operator
 * @param   filterType - control point filter type
 * @param   param1 - first filter
 * @param   param2 - second filter (if applicable), otherwise NULL

 *
 * @return  status of write
 */

uint8 glucoseCtlPntWriteFilter(uint8 opcode, uint8 oper, uint8 filterType,
                               void* param1, void* param2)
{
  attWriteReq_t writeReq;
  UTCTimeStruct *time1, *time2;
  uint16 *seqNum1, *seqNum2;
  
  uint8 *p = writeReq.value;
  
  *p++ = opcode;
  *p++ = oper;

  // The operator will tells us whether to include the filters or not
  // Note day and month are converted to date time struct values
  switch(oper)
  {
  case CTL_PNT_OPER_LESS_EQUAL:
  case CTL_PNT_OPER_GREATER_EQUAL:
    *p++ = filterType;
    if (filterType == CTL_PNT_FILTER_SEQNUM)
    {
      seqNum1 = param1;
      *p++ = LO_UINT16(*seqNum1);
      *p++ = HI_UINT16(*seqNum1);
    }
    else
    {
      time1 = param1;
      *p++ = LO_UINT16(time1->year);
      *p++ = HI_UINT16(time1->year);
      *p++ = (time1->month + 1);
      *p++ = (time1->day + 1);
      *p++ = time1->hour;
      *p++ = time1->minutes;
      *p++ = time1->seconds;
    }
    break;
  case CTL_PNT_OPER_RANGE:
    *p++ = filterType;
    if (filterType == CTL_PNT_FILTER_SEQNUM)
    {
      seqNum1 = param1;
      seqNum2 = param2;
      *p++ = LO_UINT16(*seqNum1);
      *p++ = HI_UINT16(*seqNum1);
      *p++ = LO_UINT16(*seqNum2);
      *p++ = HI_UINT16(*seqNum2);      
    }
    else
    {
      time1 = param1;
      time2 = param2;
      *p++ = LO_UINT16(time1->year);
      *p++ = HI_UINT16(time1->year);
      *p++ = (time1->month + 1);
      *p++ = (time1->day + 1);
      *p++ = time1->hour;
      *p++ = time1->minutes;
      *p++ = time1->seconds;
      
      *p++ = LO_UINT16(time2->year);
      *p++ = HI_UINT16(time2->year);
      *p++ = (time2->month + 1);
      *p++ = (time2->day + 1);
      *p++ = time2->hour;
      *p++ = time2->minutes;
      *p++ = time2->seconds;
    }
    break;
  default:
    break;
  }
  
  writeReq.len = (p - writeReq.value);
  writeReq.sig = 0;
  writeReq.cmd = 0;
  
  writeReq.handle = glucoseHdlCache[HDL_GLUCOSE_CTL_PNT_START];
  return GATT_WriteCharValue( glucCollConnHandle, &writeReq, glucCollTaskId );
}
/*********************************************************************
 * @fn      glucoseCtlPntGattMsg()
 *
 * @brief   Handle GATT messages for control point operations.
 *
 * @param   pMsg - GATT message.
 *
 * @return  None.
 */
void glucoseCtlPntGattMsg( gattMsgEvent_t *pMsg )
{ 
  if (pMsg->method == ATT_ERROR_RSP)
  {
     glucCollClearPending = false;
     LCD_WRITE_STRING("Write Error",  HAL_LCD_LINE_1);
     LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
     LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
  }
  else if (pMsg->method == ATT_WRITE_RSP)
  {
    // start procedure timer
    osal_start_timerEx( glucCollTaskId, PROCEDURE_TIMEOUT_EVT, GLUCOSE_PROCEDURE_TIMEOUT );  
  }
  
  glucCollWritePending = false;
}
