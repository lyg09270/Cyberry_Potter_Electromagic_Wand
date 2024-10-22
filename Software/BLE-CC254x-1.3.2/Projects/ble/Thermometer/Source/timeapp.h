/**************************************************************************************************
  Filename:       timeapp.h
  Revised:        $Date: 2012-02-07 12:34:04 -0800 (Tue, 07 Feb 2012) $
  Revision:       $Revision: 29163 $

  Description:    This file contains the Time App sample application
                  definitions and prototypes.

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

#ifndef TIMEAPP_H
#define TIMEAPP_H

#include "gatt.h"

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

// Time App discovery states
enum
{
  DISC_IDLE = 0x00,                       // Idle state
  DISC_CURR_TIME_START = 0x10,            // Current time service
  DISC_CURR_TIME_SVC,                     // Discover service
  DISC_CURR_TIME_CHAR,                    // Discover all characteristics
  DISC_CURR_TIME_CT_TIME_CCCD,            // Discover CT time CCCD
  DISC_FAILED = 0xFF                      // Discovery failed
};

// Time App handle cache indexes
enum
{
  HDL_CURR_TIME_CT_TIME_START,            // Current time start handle
  HDL_CURR_TIME_CT_TIME_END,              // Current time end handle
  HDL_CURR_TIME_CT_TIME_CCCD,             // Current time CCCD
  HDL_CACHE_LEN
};

// Configuration states
#define TIMEAPP_CONFIG_START              0x00
#define TIMEAPP_CONFIG_CMPL               0xFF

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * GLOBAL
 */

// Task ID
extern uint8 thermometerTaskId;

// Connection handle
extern uint16 gapConnHandle;

// Handle cache
extern uint16 timeAppHdlCache[HDL_CACHE_LEN];

// Task ID
extern uint8 timeConfigDone;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void TimeApp_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 TimeApp_ProcessEvent( uint8 task_id, uint16 events );

/*
 * Time App clock functions
 */
extern void timeAppClockInit( void );
extern void timeAppClockSet( uint8 *pData );

/* 
 * Time App service discovery functions
 */
extern uint8 timeAppDiscStart( void );
extern uint8 timeAppDiscGattMsg( uint8 state, gattMsgEvent_t *pMsg );

/* 
 * Time App characteristic configuration functions
 */
extern uint8 timeAppConfigNext( uint8 state );
extern uint8 timeAppConfigGattMsg( uint8 state, gattMsgEvent_t *pMsg );

/* 
 * Time App indication and notification handling functions
 */
extern void timeAppIndGattMsg( gattMsgEvent_t *pMsg );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TIMEAPP_H */
