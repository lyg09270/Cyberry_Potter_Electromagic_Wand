/**************************************************************************************************
  Filename:       glucose_ind.c
  Revised:        $Date: 2011-11-22 16:33:58 -0800 (Tue, 22 Nov 2011) $
  Revision:       $Revision: 54 $

  Description:    Glucose Collector App indication and notification handling routines.

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


/*********************************************************************
 * CONSTANTS
 */
#define STR_MG_PER_DL "mg/dL:"
#define STR_MMOL_PER_L "mmol/L:"

/*********************************************************************
 * TYPEDEFS
 */

// Data in a glucose measurement as defined in the profile
typedef struct {
  uint8 flags;
  uint16 seqNum;
  uint8 baseTime[7];
  int16 timeOffset;
  uint16 concentration;
  uint8 typeSampleLocation;
  uint16 sensorStatus;
} glucoseMeas_t;

// Context data as defined in profile
typedef struct {
  uint8 flags;
  uint16 seqNum;
  uint8 extendedFlags;
  uint8 carboId;
  uint16 carboVal;
  uint8 mealVal;
  uint8 TesterHealthVal;
  uint16 exerciseDuration;
  uint8 exerciseIntensity;
  uint8 medId;
  uint16 medVal;
  uint16 HbA1cVal;
} glucoseContext_t;

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

// For test purposes
static glucoseMeas_t glucoseMeas;
static glucoseContext_t glucoseContext;

/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*********************************************************************
 * @fn      glucoseIndGattMsg
 *
 * @brief   Handle indications and notifications. 
 *
 * @param   pMsg - GATT message.
 *
 * @return  none
 */
void glucoseIndGattMsg( gattMsgEvent_t *pMsg )
{
  uint8 i;
  
  // Look up the handle in the handle cache
  for ( i = 0; i < HDL_CACHE_LEN; i++ )
  {
    if ( pMsg->msg.handleValueInd.handle == glucoseHdlCache[i] )
    {
      break;
    }
  }

  // Perform processing for this handle 
  switch ( i )
  {
    case HDL_GLUCOSE_START:     
      {
        uint8* p = pMsg->msg.handleValueNoti.value;

        // restart procedure timer
        if (glucCollWritePending == true)
        {
          osal_start_timerEx( glucCollTaskId, PROCEDURE_TIMEOUT_EVT, GLUCOSE_PROCEDURE_TIMEOUT );          
        }
        
        memset(&glucoseMeas, 0, sizeof(glucoseMeas));
        
        // Flags
        glucoseMeas.flags = *p++;
        
        // Sequence number
        glucoseMeas.seqNum = BUILD_UINT16(p[0], p[1]);
        LCD_WRITE_STRING_VALUE( "SeqNum:", glucoseMeas.seqNum, 10, HAL_LCD_LINE_1 );
        p += 2;
        
        // Base time
        memcpy(glucoseMeas.baseTime, p, 7);
        p += 7;
       
        // Time offset;
        if (glucoseMeas.flags & GLUCOSE_MEAS_FLAG_TIME_OFFSET)
        {
          glucoseMeas.timeOffset = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }
        
        // Glucose concentration
        if(glucoseMeas.flags & GLUCOSE_MEAS_FLAG_CONCENTRATION)
        {
          glucoseMeas.concentration = BUILD_UINT16(p[0], p[1]);
          if(glucoseMeas.flags & GLUCOSE_MEAS_FLAG_UNITS)
            LCD_WRITE_STRING_VALUE( STR_MMOL_PER_L, glucoseMeas.concentration, 10, HAL_LCD_LINE_2 );
          else
            LCD_WRITE_STRING_VALUE( STR_MG_PER_DL, glucoseMeas.concentration, 10, HAL_LCD_LINE_2 );
          p += 2;
          
          // Type sample location
          glucoseMeas.typeSampleLocation = *p++;
        }
        
        // Sensor status annunciation
        if (glucoseMeas.flags & GLUCOSE_MEAS_FLAG_STATUS_ANNUNCIATION)
        {
          glucoseMeas.sensorStatus = BUILD_UINT16(p[0], p[1]);
          p += 2;
        }
      }
      break;
      
    case HDL_GLUCOSE_CONTEXT_START:  
      {
        uint8* p = pMsg->msg.handleValueNoti.value;

        // restart procedure timer
        if (glucCollWritePending == true)
        {
          osal_start_timerEx( glucCollTaskId, PROCEDURE_TIMEOUT_EVT, GLUCOSE_PROCEDURE_TIMEOUT );          
        }
        
        memset(&glucoseContext, 0, sizeof(glucoseContext));
        
        // Flags
        glucoseContext.flags = *p++;

        // Sequence number
        glucoseContext.seqNum = BUILD_UINT16(p[0], p[1]);
        p += 2;

        // Extended flags
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_EXTENDED)
        {
          glucoseContext.extendedFlags = *p++;
        }
        
        // Carbohydrate
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_CARBO)
        {
          // carbohydrate ID
          glucoseContext.carboId = *p++;
          
          // Carbohydrate
          glucoseContext.carboVal = BUILD_UINT16(p[0], p[1]);          
          p += 2;
        }
        
        // Meal
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_MEAL)
        {
          glucoseContext.mealVal = *p++;
        }
        
        // Tester health
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_TESTER_HEALTH)
        {
          glucoseContext.TesterHealthVal = *p++;
        }
        
        // Exercise
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_EXERCISE)
        {
          // Duration
          glucoseContext.exerciseDuration = BUILD_UINT16(p[0], p[1]);              
          p += 2;
          
          // Intensity
          glucoseContext.exerciseIntensity = *p++;
        }
        
        // Medication
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_MEDICATION)
        {
          // Medication ID
          glucoseContext.medId = *p++;

          // Medication
          glucoseContext.medVal = BUILD_UINT16(p[0], p[1]);  
          p += 2;
        }
        
        // HbA1c
        if(glucoseContext.flags & GLUCOSE_CONTEXT_FLAG_HbA1c)
        {
          glucoseContext.HbA1cVal = BUILD_UINT16(p[0], p[1]);             
          LCD_WRITE_STRING_VALUE( "HbA1c:",  glucoseContext.HbA1cVal, 10, HAL_LCD_LINE_3 );
          p += 2;
        }
        
      }
      break;
      
    case HDL_GLUCOSE_CTL_PNT_START:
      {
        uint8* pValue = pMsg->msg.handleValueInd.value;

        // stop procedure timer
        osal_stop_timerEx( glucCollTaskId, PROCEDURE_TIMEOUT_EVT );  
        
        if(pValue[0] == CTL_PNT_OP_NUM_RSP)
        {
          if(pMsg->msg.handleValueInd.len >= 3)
          {
            LCD_WRITE_STRING("Matching ",  HAL_LCD_LINE_1);
            LCD_WRITE_STRING( "Records:", HAL_LCD_LINE_2 );
            LCD_WRITE_STRING_VALUE("", BUILD_UINT16(pValue[2], pValue[3]), 10, HAL_LCD_LINE_3 );
          }
        }
        else if(pValue[0] == CTL_PNT_OP_REQ_RSP && glucCollClearPending)
        {
          glucCollClearPending = false;
          
          if(pMsg->msg.handleValueInd.len >= 3)
          {
            switch(pValue[3])
            {
            case CTL_PNT_RSP_SUCCESS:
              LCD_WRITE_STRING("Records",  HAL_LCD_LINE_1);
              LCD_WRITE_STRING("Cleared", HAL_LCD_LINE_2 );
              LCD_WRITE_STRING("", HAL_LCD_LINE_3 );
              break;
            case CTL_PNT_RSP_NO_RECORDS:
              LCD_WRITE_STRING("No Matching",  HAL_LCD_LINE_1);
              LCD_WRITE_STRING("Records", HAL_LCD_LINE_2 );
              LCD_WRITE_STRING("to Delete", HAL_LCD_LINE_3 );
              break;
            default:
              LCD_WRITE_STRING("Error:",  HAL_LCD_LINE_1);
              LCD_WRITE_STRING_VALUE("", pValue[3], 10, HAL_LCD_LINE_2 );  
              LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
              break;
            }
          }
        }
        else if(pValue[0] == CTL_PNT_OP_REQ_RSP)
        {
          if(pMsg->msg.handleValueInd.len >= 3)
          {
            switch(pValue[3])
            {
            case CTL_PNT_RSP_SUCCESS:
              break;
            case CTL_PNT_RSP_NO_RECORDS:
              LCD_WRITE_STRING("No Matching",  HAL_LCD_LINE_1);
              LCD_WRITE_STRING("Records", HAL_LCD_LINE_2 );
              LCD_WRITE_STRING("Found", HAL_LCD_LINE_3 );
              break;
            default:
              LCD_WRITE_STRING("Error:",  HAL_LCD_LINE_1);
              LCD_WRITE_STRING_VALUE("", pValue[3], 10, HAL_LCD_LINE_2 );  
              LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
              break;
            }
          }
        }
      }
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
