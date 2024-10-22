/**************************************************************************************************
  Filename:       timeapp.c
  Revised:        $Date: 2012-01-16 13:29:21 -0800 (Mon, 16 Jan 2012) $
  Revision:       $Revision: 62 $

  Description:    This file contains the time app sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

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
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "ll.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "timeapp.h"
#include "battservice.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Delay to begin discovery from start of connection in ms
#define DEFAULT_DISCOVERY_DELAY               1000

// Clock display update period in ms; set to 60 sec or less since time is displayed in minutes
#define DEFAULT_CLOCK_UPDATE_PERIOD           10000

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Reference Time Update Service values for time update control point
#define REF_TIME_UPDATE_GET                   0x01    // Get Reference Update
#define REF_TIME_UPDATE_CANCEL                0x02    // Cancel Reference Update

// Ringer control point
#define RINGER_SILENT_MODE                    1       // Silent Mode
#define RINGER_MUTE_ONCE                      2       // Mute Once
#define RINGER_CANCEL_SILENT                  3       // Cancel Silent Mode 

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 timeAppTaskId;

// Connection handle
uint16 timeAppConnHandle;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static gaprole_States_t timeAppGapState = GAPROLE_INIT;

// Service discovery state
static uint8 timeAppDiscState = DISC_IDLE;

// Service discovery complete
static uint8 timeAppDiscoveryCmpl = FALSE;

// Characteristic configuration state
static uint8 timeAppConfigState = TIMEAPP_CONFIG_START;

// TRUE if pairing started
static uint8 timeAppPairingStarted = FALSE;

// TRUE if discovery postponed due to pairing
static uint8 timeAppDiscPostponed = FALSE;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 timeAppScanData[] =
{
  // local name
  0x09,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T',
  'i',
  'm',
  'e',
  ' ',
  'A',
  'p',
  'p'
};

static uint8 timeAppAdvData[] = 
{
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service solicitation
  0x0D,
  GAP_ADTYPE_SERVICES_LIST_16BIT,
  LO_UINT16(CURRENT_TIME_SVC_UUID),
  HI_UINT16(CURRENT_TIME_SVC_UUID),
  LO_UINT16(REF_TIME_UPDATE_SVC_UUID),
  HI_UINT16(REF_TIME_UPDATE_SVC_UUID),
  LO_UINT16(DST_CHANGE_SVC_UUID),
  HI_UINT16(DST_CHANGE_SVC_UUID),
  LO_UINT16(NWA_SVC_UUID),
  HI_UINT16(NWA_SVC_UUID),
  LO_UINT16(ALERT_NOTIF_SVC_UUID),
  HI_UINT16(ALERT_NOTIF_SVC_UUID),
  LO_UINT16(BATT_SERVICE_UUID),
  HI_UINT16(BATT_SERVICE_UUID)
};

// Device name attribute value
static uint8 timeAppDeviceName[GAP_DEVICE_NAME_LEN] = "Time App";

// Bonded state
static bool timeAppBonded = FALSE;

// Bonded peer address
static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Reference time update value
static uint8 timeAppRefUpdateVal = REF_TIME_UPDATE_GET;

// Alert notification control point command test values
static const uint8 timeAppAlertCmd[] =
{
  ALERT_NOTIF_ENABLE_NEW,
  ALERT_NOTIF_ENABLE_UNREAD,
  ALERT_NOTIF_NOTIFY_NEW,
  ALERT_NOTIF_NOTIFY_UNREAD,
  ALERT_NOTIF_DISABLE_NEW,
  ALERT_NOTIF_DISABLE_UNREAD
};

// Pointer to above command test values
static uint8 const *pTimeAppAlertCmd = timeAppAlertCmd;

// Advertising user-cancelled state
static bool timeAppAdvCancelled = FALSE;

// Ringer control point test value
static uint8 timeAppRingerCmd = RINGER_SILENT_MODE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void timeAppProcessGattMsg( gattMsgEvent_t *pMsg );
static void timeAppGapStateCB( gaprole_States_t newState );
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void timeApp_HandleKeys( uint8 shift, uint8 keys );
static void timeApp_ProcessOSALMsg( osal_event_hdr_t *pMsg );

#ifndef CC2540_MINIDK
static char *bdAddr2Str( uint8 *pAddr );
#endif 
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t timeAppPeripheralCB =
{
  timeAppGapStateCB,              // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t timeAppBondCB =
{
  timeAppPasscodeCB,
  timeAppPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      TimeApp_Init
 *
 * @brief   Initialization function for the Time App App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void TimeApp_Init( uint8 task_id )
{
  timeAppTaskId = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    uint8 advEnable = FALSE;      
    uint16 advertOffTime = 0;
    uint8 updateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 minInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 maxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 slaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &advertOffTime );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &updateRequest );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &minInterval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &maxInterval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &slaveLatency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &connTimeout );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( timeAppScanData ), timeAppScanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( timeAppAdvData ), timeAppAdvData );
  }
  
  // Setup GAP
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, timeAppDeviceName );

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
  GATT_RegisterForInd( timeAppTaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( timeAppTaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // If key pressed on startup clear all stored bonds
  if ( HalKeyRead() & HAL_KEY_CENTER )
  {
    GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, 0 );
  }
  
  // Start clock update timer
  osal_start_timerEx( timeAppTaskId, CLOCK_UPDATE_EVT, DEFAULT_CLOCK_UPDATE_PERIOD );
  
  // Setup a delayed profile startup
  osal_set_event( timeAppTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      TimeApp_ProcessEvent
 *
 * @brief   Time App Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 TimeApp_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( timeAppTaskId )) != NULL )
    {
      timeApp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &timeAppPeripheralCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &timeAppBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    if ( timeAppPairingStarted )
    {
      // Postpone discovery until pairing completes
      timeAppDiscPostponed = TRUE;
    }
    else
    {
      timeAppDiscState = timeAppDiscStart();
    }  
    return ( events ^ START_DISCOVERY_EVT );
  }

  if ( events & CLOCK_UPDATE_EVT )
  {
    timeAppClockDisplay();

    // Restart clock tick timer
    osal_start_timerEx( timeAppTaskId, CLOCK_UPDATE_EVT, DEFAULT_CLOCK_UPDATE_PERIOD );

    return ( events ^ CLOCK_UPDATE_EVT );
  }
     
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      timeApp_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void timeApp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      timeApp_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      timeAppProcessGattMsg( (gattMsgEvent_t *) pMsg );
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      timeApp_HandleKeys
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
static void timeApp_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_UP )
  {
    // Start or stop advertising
    if ( timeAppGapState != GAPROLE_CONNECTED )
    {
      uint8 advState;
      
      // Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );

      // Toggle advertising state      
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &advState );
      advState = !advState;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
      
      // Set state variable
      if (advState == FALSE)
      {
        timeAppAdvCancelled = TRUE;
      }
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {
    attWriteReq_t req;

    // Send command to alert notificaton control point
    if ( ( timeAppGapState == GAPROLE_CONNECTED ) &&
         ( timeAppHdlCache[HDL_ALERT_NTF_CTRL] != 0 ) )
    {
      // Send write request
      req.len = 2;
      req.value[0] = *pTimeAppAlertCmd;
      req.value[1] = ALERT_NOTIF_CAT_ALL;
      req.sig = 0;
      req.cmd = 0;
      req.handle = timeAppHdlCache[HDL_ALERT_NTF_CTRL];
      GATT_WriteCharValue( timeAppConnHandle, &req, timeAppTaskId );
      LCD_WRITE_STRING_VALUE( "Alert cmd:", *pTimeAppAlertCmd, 10, HAL_LCD_LINE_1);
    }
    
    // Cycle through command test values
    if ( *pTimeAppAlertCmd == ALERT_NOTIF_DISABLE_UNREAD )
    {
      pTimeAppAlertCmd = timeAppAlertCmd;
    }
    else
    {
      pTimeAppAlertCmd++;
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
    attWriteReq_t req;
    
    // Do a reference time update
    if ( ( timeAppGapState == GAPROLE_CONNECTED ) &&
         ( timeAppHdlCache[HDL_REF_TIME_UPD_CTRL] != 0 ) )
    {
      // Send write command
      req.len = 1;
      req.value[0] = timeAppRefUpdateVal;
      req.sig = 0;
      req.cmd = 1;
      req.handle = timeAppHdlCache[HDL_REF_TIME_UPD_CTRL];
      GATT_WriteNoRsp( timeAppConnHandle, &req );

      LCD_WRITE_STRING_VALUE( "Time update:", timeAppRefUpdateVal, 10, HAL_LCD_LINE_1);
      
      // Toggle between two reference time update values
      if ( timeAppRefUpdateVal == REF_TIME_UPDATE_GET )
      {
        timeAppRefUpdateVal = REF_TIME_UPDATE_CANCEL;
      }
      else
      {
        timeAppRefUpdateVal = REF_TIME_UPDATE_GET;
      }
    }
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
    // If connected, terminate connection
    if ( timeAppGapState == GAPROLE_CONNECTED )
    {
      GAPRole_TerminateConnection();
    }
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
    attWriteReq_t req;
    
    // Write ringer control point
    if ( ( timeAppGapState == GAPROLE_CONNECTED ) &&
         ( timeAppHdlCache[HDL_PAS_CTRL] != 0 ) )
    {
      // Send write command
      req.len = 1;
      req.value[0] = timeAppRingerCmd;
      req.sig = 0;
      req.cmd = 1;
      req.handle = timeAppHdlCache[HDL_PAS_CTRL];
      GATT_WriteNoRsp( timeAppConnHandle, &req );

      LCD_WRITE_STRING_VALUE( "Ringer ctrl:", timeAppRingerCmd, 10, HAL_LCD_LINE_1);
      
      // Toggle between values
      if ( ++timeAppRingerCmd > RINGER_CANCEL_SILENT )
      {
        timeAppRingerCmd = RINGER_SILENT_MODE;
      }
    }
  }
  
  if ( keys & HAL_KEY_SW_6 )
  {

  }

}

/*********************************************************************
 * @fn      timeAppProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void timeAppProcessGattMsg( gattMsgEvent_t *pMsg )
{
  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    timeAppIndGattMsg( pMsg );
  }
  else if ( ( pMsg->method == ATT_READ_RSP || pMsg->method == ATT_WRITE_RSP ) ||
            ( pMsg->method == ATT_ERROR_RSP &&
              ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ||
                pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    timeAppConfigState = timeAppConfigGattMsg ( timeAppConfigState, pMsg );
    if ( timeAppConfigState == TIMEAPP_CONFIG_CMPL )
    {
      timeAppDiscoveryCmpl = TRUE;
    }
  }
  else
  {
    timeAppDiscState = timeAppDiscGattMsg( timeAppDiscState, pMsg );
    if ( timeAppDiscState == DISC_IDLE )
    {      
      // Start characteristic configuration
      timeAppConfigState = timeAppConfigNext( TIMEAPP_CONFIG_START );
    }
  }
}

/*********************************************************************
 * @fn      timeAppDisconnected
 *
 * @brief   Handle disconnect. 
 *
 * @return  none
 */
static void timeAppDisconnected( void )
{
  // Initialize state variables
  timeAppDiscState = DISC_IDLE;
  timeAppPairingStarted = FALSE;
  timeAppDiscPostponed = FALSE;
}

/*********************************************************************
 * @fn      timeAppGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void timeAppGapStateCB( gaprole_States_t newState )
{
  // if connected
  if ( newState == GAPROLE_CONNECTED )
  {
    linkDBItem_t  *pItem;

    // Get connection handle
    GAPRole_GetParameter( GAPROLE_CONNHANDLE, &timeAppConnHandle );

    // Get peer bd address
    if ( (pItem = linkDB_Find( timeAppConnHandle )) != NULL)
    {
      // If connected to device without bond do service discovery
      if ( !osal_memcmp( pItem->addr, timeAppBondedAddr, B_ADDR_LEN ) )
      {
        timeAppDiscoveryCmpl = FALSE;
      }
      
      // Initiate service discovery if necessary
      if ( timeAppDiscoveryCmpl == FALSE )
      {
        osal_start_timerEx( timeAppTaskId, START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
      }
      // Perform configuration of characteristics on connection setup
      else
      {
        timeAppConfigState = timeAppConfigNext( TIMEAPP_CONFIG_CONN_START );
      }
      
    }
      
    LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
  }
  // if disconnected
  else if ( timeAppGapState == GAPROLE_CONNECTED && 
            newState != GAPROLE_CONNECTED )
  {
    uint8 advState = TRUE;
      
    timeAppDisconnected();
    
    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    
    LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
  }
  // if advertising stopped
  else if ( timeAppGapState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( timeAppAdvCancelled )
    {
      timeAppAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    }  
  }
  // if started
  else if ( newState == GAPROLE_STARTED )
  {
    
#ifndef CC2540_MINIDK    
    uint8 bdAddr[B_ADDR_LEN];
    
    GAPRole_GetParameter( GAPROLE_BD_ADDR, &bdAddr );
    LCD_WRITE_STRING( "Time App", HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( bdAddr2Str( bdAddr ),  HAL_LCD_LINE_2 );
#endif 
    
    // Initialize time clock after writing first two lines of LCD
    timeAppClockInit();
  }
  
  timeAppGapState = newState;
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    timeAppPairingStarted = TRUE;
    
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    timeAppPairingStarted = FALSE;

    if ( status == SUCCESS )
    {
      linkDBItem_t  *pItem;
      
      if ( (pItem = linkDB_Find( timeAppConnHandle )) != NULL )
      {
        // Store bonding state of pairing
        timeAppBonded = ( (pItem->stateFlags & LINK_BOUND) == LINK_BOUND );
        
        if ( timeAppBonded )
        {
          osal_memcpy( timeAppBondedAddr, pItem->addr, B_ADDR_LEN );
        }
      }
      
      // If discovery was postponed start discovery
      if ( timeAppDiscPostponed && timeAppDiscoveryCmpl == FALSE )
      {
        timeAppDiscPostponed = FALSE;
        osal_set_event( timeAppTaskId, START_DISCOVERY_EVT );
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
 * @fn      timeAppPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ) );
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa( passcode, str, 10 ),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
#ifndef CC2540_MINIDK
static char *bdAddr2Str( uint8 *pAddr )
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
#endif

/*********************************************************************
*********************************************************************/
