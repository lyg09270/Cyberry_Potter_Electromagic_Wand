/**************************************************************************************************
  Filename:       glucoseCollector.h
  Revised:        $Date: 2011-11-22 16:33:58 -0800 (Tue, 22 Nov 2011) $
  Revision:       $Revision: 54 $

  Description:    This file contains the Glucose Collector sample application
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

#ifndef GLUCOSECOLLECTOR_H
#define GLUCOSECOLLECTOR_H

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

// Glucose App discovery states
enum
{
  DISC_IDLE = 0x00,                       // Idle state
  
  DISC_GLUCOSE_START = 0x10,            // Current glucose service
  DISC_GLUCOSE_SVC,                     // Discover service
  DISC_GLUCOSE_CHAR,                    // Discover all characteristics
  DISC_GLUCOSE_CCCD,                    // Discover glucose CCCD
  DISC_GLUCOSE_CONTEXT_CCCD,            // Discover context CCCD
  DISC_GLUCOSE_CTL_PNT_CCCD,            // Discover record control point CCCD  
  
  DISC_DEVINFO_START = 0x20,
  DISC_DEVINFO_SVC,
  DISC_DEVINFO_CHAR,
 
  DISC_FAILED = 0xFF                      // Discovery failed
};


// Glucose handle cache indexes
enum
{
  HDL_GLUCOSE_START,
  HDL_GLUCOSE_END,
  HDL_GLUCOSE_MEAS_CCCD,
  HDL_GLUCOSE_CONTEXT_START,
  HDL_GLUCOSE_CONTEXT_END,
  HDL_GLUCOSE_CONTEXT_CCCD,
  HDL_GLUCOSE_FEATURE,
  HDL_GLUCOSE_CTL_PNT_START,
  HDL_GLUCOSE_CTL_PNT_END,
  HDL_GLUCOSE_CTL_PNT_CCCD,
  HDL_DEVINFO_SYSTEM_ID,
  HDL_DEVINFO_MODEL_NUM,
  HDL_DEVINFO_MANUFACTURER_NAME,
  
  HDL_CACHE_LEN
};

// Configuration states
#define GLUCOSE_CONFIG_START              0x00
#define GLUCOSE_CONFIG_CMPL               0xFF
  
// Glucose Task Events
#define START_DEVICE_EVT                              0x0001
#define START_DISCOVERY_EVT                           0x0002
#define PROCEDURE_TIMEOUT_EVT                         0x0004

/*********************************************************************
 * MACROS
 */

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task ID
extern uint8 glucCollTaskId;

// Connection handle
extern uint16 glucCollConnHandle;

// Features
extern uint16 glucoseFeatures;

// Handle cache
extern uint16 glucoseHdlCache[HDL_CACHE_LEN];

// control point write in progess
extern bool glucCollWritePending;

// control point clear in progress
extern bool glucCollClearPending;


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void GlucColl_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 GlucColl_ProcessEvent( uint8 task_id, uint16 events );


/* 
 * Glucose service discovery functions
 */
extern uint8 glucoseDiscStart( void );
extern uint8 glucoseDiscGattMsg( uint8 state, gattMsgEvent_t *pMsg );

/* 
 * Glucose characteristic configuration functions
 */
extern uint8 glucoseConfigNext( uint8 state );
extern uint8 glucoseConfigGattMsg( uint8 state, gattMsgEvent_t *pMsg );

/* 
 * Glucose indication and notification handling functions
 */
extern void glucoseIndGattMsg( gattMsgEvent_t *pMsg );

/* 
 * Glucose control point functions
 */
extern uint8 glucoseCtlPntWrite(uint8 opcode, uint8 oper);
extern uint8 glucoseCtlPntWriteFilter(uint8 opcode, uint8 oper, uint8 filterType,
                                      void* param1, void* param2);
extern void glucoseCtlPntGattMsg( gattMsgEvent_t *pMsg );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GLUCOSECOLLECTOR_H */
