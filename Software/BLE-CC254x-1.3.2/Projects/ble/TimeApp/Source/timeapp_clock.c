/**************************************************************************************************
  Filename:       timeapp_clock.c
  Revised:        $Date: 2011-03-23 15:19:04 -0700 (Wed, 23 Mar 2011) $
  Revision:       $Revision: 15 $

  Description:    Time clock display and timekeeping for Time App.

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
#include "OSAL_Clock.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "timeapp.h"

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

// Month string
static const char timeAppMonthStr[12][3] =
{
  {'J', 'a', 'n'},
  {'F', 'e', 'b'},
  {'M', 'a', 'r'},
  {'A', 'p', 'r'},
  {'M', 'a', 'y'},
  {'J', 'u', 'n'},
  {'J', 'u', 'l'},
  {'A', 'u', 'g'},
  {'S', 'e', 'p'},
  {'O', 'c', 't'},
  {'N', 'o', 'v'},
  {'D', 'e', 'c'}
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static char *num2Str( char *pStr, uint8 num );

/*********************************************************************
 * @fn      num2Str()
 *
 * @brief   Convert unsigned int 0-99 to decimal digit string.
 *
 * @return  pointer to string
 */
static char *num2Str( char *pStr, uint8 num )
{
  *pStr++ = (num / 10) + '0';
  *pStr++ = (num % 10) + '0';
  
  return pStr;
}

/*********************************************************************
 * @fn      timeAppClockDisplay()
 *
 * @brief   Write the clock time to the display.
 *
 * @return  none
 */
void timeAppClockDisplay( void )
{
  char          displayBuf[HAL_LCD_MAX_CHARS];
  char          *p = displayBuf;
  UTCTimeStruct time;
  
  // Get time structure from OSAL
  osal_ConvertUTCTime( &time, osal_getClock() );
  
  // Display is in the format:
  // HH:MM MmmDD YYYY
  
  p = num2Str( p, time.hour );
  *p++ = ':';
  p = num2Str( p, time.minutes );
  *p++ = ' ';

  *p++ = timeAppMonthStr[time.month][0];  
  *p++ = timeAppMonthStr[time.month][1];
  *p++ = timeAppMonthStr[time.month][2];

  p = num2Str( p, time.day + 1 );
  *p++ = ' ';
  
  _ltoa( time.year, (uint8 *) p, 10 );
  
  LCD_WRITE_STRING( displayBuf, HAL_LCD_LINE_3 );  
}

/*********************************************************************
 * @fn      timeAppClockInit()
 *
 * @brief   Initialize the Time App clock.
 *
 * @return  none
 */
void timeAppClockInit( void )
{
  // Update the clock display
  timeAppClockDisplay();
}

/*********************************************************************
 * @fn      timeAppClockSet()
 *
 * @brief   Set the clock. 
 *
 * @param   pData - Pointer to a Date Time characteristic structure
 *
 * @return  none
 */
void timeAppClockSet( uint8 *pData )
{
  UTCTimeStruct time;
  
  // Parse time service structure to OSAL time structure
  time.year = BUILD_UINT16(pData[0], pData[1]);
  if ( time.year == 0 )
  {
    time.year = 2000;
  }
  pData += 2;
  time.month = *pData++;
  if ( time.month > 0 )
  {
    time.month--;
  }
  time.day = *pData++;
  if ( time.day > 0 )
  {
    time.day--;
  }
  time.hour = *pData++;
  time.minutes = *pData++;
  time.seconds = *pData;
  
  // Update OSAL time
  osal_setClock( osal_ConvertUTCSecs( &time ) );
  
  timeAppClockDisplay();
}

/*********************************************************************
*********************************************************************/
