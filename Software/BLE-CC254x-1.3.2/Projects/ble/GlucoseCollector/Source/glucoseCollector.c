/**************************************************************************************************
  Filename:       glucoseCollector.c
  Revised:        $Date: 2013-05-09 11:27:04 -0700 (Thu, 09 May 2013) $
  Revision:       $Revision: 34210 $

  Description:    This file contains the Glucose Collector sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
#include "OSAL_PwrMgr.h"
#include "OSAL_clock.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "glucoseCollector.h"
#include "glucservice.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Set to TRUE to filter record requests, FALSE to request/delete all
#define GLUCOSE_FILTER_ENABLED                FALSE

// Filter by time or sequence number: CTL_PNT_FILTER_SEQNUM or CTL_PNT_FILTER_TIME
#define DEFAULT_FILTER_TYPE                   CTL_PNT_FILTER_SEQNUM

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE //FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_DISPLAY //GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID for internal task/event processing
uint8 glucCollTaskId;

// Connection handle of current connection 
uint16 glucCollConnHandle = GAP_CONNHANDLE_INIT;

uint16 glucoseFeatures = 0;

// Discovered characteristic handles
bool glucCollCharHdls = false;

bool glucCollWritePending = false;

bool glucCollClearPending = false;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */



#if GLUCOSE_FILTER_ENABLED
#if DEFAULT_FILTER_TYPE == CTL_PNT_FILTER_TIME
// These filters should return the first two records in the sample app only.
// Note dates are in UTC time; day and month start at 0
static UTCTimeStruct filterTime1 = {0,0,0,0,0,2011};
static UTCTimeStruct filterTime2 = {0,0,0,27,1,2011};
static void *pFilter1 = &filterTime1;
static void *pFilter2 = &filterTime2;
#else
// Sequence number filters
static uint16 filterSeqNum1 = 1;
static uint16 filterSeqnum2 = 10;
static void *pFilter1 = &filterSeqNum1;
static void *pFilter2 = &filterSeqnum2;
#endif
#endif

// GAP GATT Attributes
static const uint8 glucCollDeviceName[GAP_DEVICE_NAME_LEN] = "Glucose Collector";

// Number of scan results and scan result index
static uint8 glucCollScanRes;
static uint8 glucCollScanIdx;

// Scan result list
static gapDevRec_t glucCollDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 glucCollScanning = FALSE;

// Application state
static uint8 glucCollState = BLE_STATE_IDLE;

// Discovery state
static uint8 glucCollDiscState = DISC_IDLE;

// Characteristic configuration state
static uint8 glucCollConfigState = GLUCOSE_CONFIG_START;

// TRUE if pairing started
static uint8 glucCollPairingStarted = FALSE;

// TRUE if discovery postponed due to pairing
static uint8 glucCollDiscPostponed = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void glucCollProcessGATTMsg( gattMsgEvent_t *pMsg );
static void glucCollCentralRssiCB( uint16 connHandle, int8  rssi );
static void glucCollCentralEventCB( gapCentralRoleEvent_t *pEvent );
static void glucCollCentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void glucCollCentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void glucColl_HandleKeys( uint8 shift, uint8 keys );
static void glucCollCentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void glucCollCentralStartDiscovery( void );
static bool glucCollFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void glucCollAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t glucCollRoleCB =
{
  glucCollCentralRssiCB,       // RSSI callback
  glucCollCentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t glucCollBondCB =
{
  glucCollCentralPasscodeCB,
  glucCollCentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GlucColl_Init
 *
 * @brief   Initialization function for the Glucose Collector App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GlucColl_Init( uint8 task_id )
{
  glucCollTaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) glucCollDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( glucCollTaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( glucCollTaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( glucCollTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      GlucColl_ProcessEvent
 *
 * @brief   Glucose Collector Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 GlucColl_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( glucCollTaskId )) != NULL )
    {
      glucCollCentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & PROCEDURE_TIMEOUT_EVT )
  {
    if ( glucCollState == BLE_STATE_CONNECTED )
    {
      // disconnect
      glucCollState = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink( glucCollConnHandle );
      
      LCD_WRITE_STRING( "Timeout", HAL_LCD_LINE_1 ); 
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
    }

    // return unprocessed events
    return ( events ^ PROCEDURE_TIMEOUT_EVT );
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &glucCollRoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &glucCollBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    if ( glucCollPairingStarted )
    {
      // Postpone discovery until pairing completes
      glucCollDiscPostponed = TRUE;
    }
    else
    {
      glucCollCentralStartDiscovery( );
    }  
    return ( events ^ START_DISCOVERY_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      glucCollCentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void glucCollCentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      glucColl_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      glucCollProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      glucColl_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
static void glucColl_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery
    if ( glucCollState != BLE_STATE_CONNECTED )
    {
      if ( !glucCollScanning )
      {
        glucCollScanning = TRUE;
        glucCollScanRes = 0;
        
        LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    else if ( glucCollState == BLE_STATE_CONNECTED &&
              glucCollCharHdls == true &&
              glucCollWritePending == false)
    {
      uint8 status;
#if GLUCOSE_FILTER_ENABLED
     // Request number of records
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_GET_NUM, CTL_PNT_OPER_RANGE,
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);      
#else
      // Request number of records
      status = glucoseCtlPntWrite(CTL_PNT_OP_GET_NUM, CTL_PNT_OPER_ALL);
#endif      
      
      if(status == 0)
        glucCollWritePending = true;
    } 
  }

  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( glucCollState != BLE_STATE_CONNECTED && !glucCollScanning && glucCollScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        glucCollScanIdx++;
        if ( glucCollScanIdx >= glucCollScanRes )
        {
          glucCollScanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE( "Device", glucCollScanIdx + 1,
                                10, HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( glucCollDevList[glucCollScanIdx].addr ),
                          HAL_LCD_LINE_2 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
    }
    else if ( glucCollState == BLE_STATE_CONNECTED )
    {
      uint8 status;

      // Abort
      status = glucoseCtlPntWrite(CTL_PNT_OP_ABORT, CTL_PNT_OPER_NULL);
      
      if(status == 0)
        glucCollWritePending = false;
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
    // Request all records
    if ( glucCollState == BLE_STATE_CONNECTED &&
         glucCollCharHdls == true &&
         glucCollWritePending == false)
    { 
      uint8 status;
#if GLUCOSE_FILTER_ENABLED
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_REQ, CTL_PNT_OPER_RANGE, 
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);
#else       
      status = glucoseCtlPntWrite(CTL_PNT_OP_REQ, CTL_PNT_OPER_ALL);
#endif
      if(status == SUCCESS)
        glucCollWritePending = true;
    }
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
    if ( glucCollState == BLE_STATE_IDLE )
    {
      // if there is a scan result
      if ( glucCollScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = glucCollDevList[glucCollScanIdx].addr;
        addrType = glucCollDevList[glucCollScanIdx].addrType;
      
        glucCollState = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
        LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
      }
    }
    else if ( glucCollState == BLE_STATE_CONNECTING ||
              glucCollState == BLE_STATE_CONNECTED )
    {
      // disconnect
      glucCollState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( glucCollConnHandle );
      
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
    }
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
    // Clear stored records
    if ( glucCollState == BLE_STATE_CONNECTED &&
         glucCollCharHdls == true &&
         glucCollWritePending == false)
    { 
      uint8 status;      
#if GLUCOSE_FILTER_ENABLED
      status = glucoseCtlPntWriteFilter(CTL_PNT_OP_CLR, CTL_PNT_OPER_RANGE,
                                        DEFAULT_FILTER_TYPE, pFilter1, pFilter2);
#else         
      status = glucoseCtlPntWrite(CTL_PNT_OP_CLR, CTL_PNT_OPER_ALL);
#endif
      if(status == 0)
      {
        glucCollWritePending = true;
        glucCollClearPending = true;
      }
    }
    else if ( glucCollState != BLE_STATE_CONNECTED )
    {
      // erase all bonds
      GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
      LCD_WRITE_STRING( "Erasing bonds", HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );

      // initiate service discovery again
      glucCollCharHdls = false;
    }
  }
}

/*********************************************************************
 * @fn      glucCollProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void glucCollProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    glucoseIndGattMsg( pMsg );
  }
  else if ( pMsg->method == ATT_READ_RSP ||
            pMsg->method == ATT_WRITE_RSP)
  {
    if(glucCollCharHdls == true)
    {
       glucoseCtlPntGattMsg(pMsg);
    }
    else
    {
      glucCollConfigState = glucoseConfigGattMsg ( glucCollConfigState, pMsg );
    }
  }
  else if(glucCollDiscState != DISC_IDLE)
  {
    glucCollDiscState = glucoseDiscGattMsg(glucCollDiscState, pMsg);  
    if ( glucCollDiscState == DISC_IDLE )
    {      
      // Start characteristic configuration
      glucCollConfigState = glucoseConfigNext( GLUCOSE_CONFIG_START );
    }
  }
  else if (pMsg->method == ATT_ERROR_RSP)
  {
    if(glucCollCharHdls == true)
    {
       glucoseCtlPntGattMsg(pMsg);
    }
    else
    {
      glucCollConfigState = glucoseConfigGattMsg ( glucCollConfigState, pMsg );
    } 
  }
}

/*********************************************************************
 * @fn      glucCollCentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void glucCollCentralRssiCB( uint16 connHandle, int8 rssi )
{
    
}

/*********************************************************************
 * @fn      glucCollCentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void glucCollCentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "Gluc. Collector", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( glucCollFindSvcUuid( GLUCOSE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            glucCollAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        glucCollScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          glucCollScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( glucCollDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", glucCollScanRes,
                                10, HAL_LCD_LINE_1 );
        if ( glucCollScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        }

        // initialize scan index to last device
        glucCollScanIdx = glucCollScanRes;

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          glucCollState = BLE_STATE_CONNECTED;
          glucCollConnHandle = pEvent->linkCmpl.connectionHandle;
          
          
          // If service discovery not performed initiate service discovery
          if ( glucCollCharHdls == false)
          {
            osal_start_timerEx( glucCollTaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );   
        }
        else
        {
          glucCollState = BLE_STATE_IDLE;
          glucCollConnHandle = GAP_CONNHANDLE_INIT;
          glucCollDiscState = DISC_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
          LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        glucCollState = BLE_STATE_IDLE;
        glucCollConnHandle = GAP_CONNHANDLE_INIT;
        glucCollDiscState = DISC_IDLE;
        glucCollPairingStarted = false;
        glucCollDiscPostponed = false;
        glucCollClearPending = false;

        // stop procedure timer
        osal_stop_timerEx( glucCollTaskId, PROCEDURE_TIMEOUT_EVT );  

        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );        
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void glucCollCentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    glucCollPairingStarted = true;
    
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    glucCollPairingStarted = false;
    
    if ( status == SUCCESS )
    {
      // If discovery was postponed start discovery
      if ( glucCollDiscPostponed &&  glucCollCharHdls == false)
      {
        glucCollDiscPostponed = false;
        osal_set_event( glucCollTaskId, START_DISCOVERY_EVT );
      }
      
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    { 
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      glucCollCentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void glucCollCentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Is the callback to get the passcode from or display it to the user?
  if ( uiInputs != 0 )
  {
    // Passcode must be entered by the user but use the default passcode for now
    passcode = DEFAULT_PASSCODE;
  }
  else
  {
    // Create random passcode
    LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  }
  
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      glucCollCentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void glucCollCentralStartDiscovery( void )
{
  glucCollDiscState = glucoseDiscStart();
}


/*********************************************************************
 * @fn      glucCollFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool glucCollFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      glucCollAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void glucCollAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( glucCollScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < glucCollScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, glucCollDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( glucCollDevList[glucCollScanRes].addr, pAddr, B_ADDR_LEN );
    glucCollDevList[glucCollScanRes].addrType = addrType;
    
    // Increment scan result count
    glucCollScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/
