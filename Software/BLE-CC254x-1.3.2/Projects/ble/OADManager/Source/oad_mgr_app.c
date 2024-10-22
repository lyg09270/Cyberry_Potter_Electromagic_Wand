/**************************************************************************************************
  Filename:       oad_mgr_app.c
  Revised:        $Date: 2012-12-10 09:54:54 -0800 (Mon, 10 Dec 2012) $
  Revision:       $Revision: 32508 $

  Description:    This file contains the OAD Manager sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

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
#include "OnBoard.h"
#include "hal_board.h"
#include "hal_flash.h"
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
#include "hal_key.h"
#endif
#if (defined HAL_LED) && (HAL_LED == TRUE)
#include "hal_led.h"
#endif
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
#include "hal_lcd.h"
#endif
#include "gatt.h"
#include "gatt_uuid.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "oad.h"
#include "oad_mgr_app.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

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

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_OAD_MIN_CONN_INTERVAL         8

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_OAD_MAX_CONN_INTERVAL         8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_OAD_SLAVE_LATENCY             0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_OAD_CONN_TIMEOUT              600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

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

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
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

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 oadManagerTaskId;

// GAP GATT Attributes
static const uint8 oadManagerDeviceName[GAP_DEVICE_NAME_LEN] = "OAD Manager";

// Number of scan results and scan result index
static uint8 oadManagerScanRes;
static uint8 oadManagerScanIdx;

// Scan result list
static gapDevRec_t oadManagerDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 oadManagerScanning = FALSE;

// Connection handle of current connection
static uint16 oadManagerConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 oadManagerState = BLE_STATE_IDLE;

static uint8 oadManagerDiscIdx;

static uint16 oadManagerHandles[OAD_CHAR_CNT];

static uint16 oadBlkNum, oadBlkTot;

static uint8 oadManagerAddr[B_ADDR_LEN] = { 0 };

// Service handles used during discovery
static uint16 oadSvcStartHdl;
static uint16 oadSvcEndHdl;

static uint8 oadManagerCCCIdx;

// CCCD handles used to enable notifications
static uint16 oadManagerCCCHandles[OAD_CHAR_CNT];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void oadManagerProcessGATTMsg( gattMsgEvent_t *pPkt );
static void oadManagerEventCB( gapCentralRoleEvent_t *pEvent );
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
static void oadManager_HandleKeys( uint8 shift, uint8 keys );
#endif
static void oadManager_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void oadManagerSvcDiscoveryMsg( gattMsgEvent_t *pMsg );
static void oadManagerCCCDiscoveryMsg( gattMsgEvent_t *pMsg );
static void oadManagerCharDiscoveryMsg( gattMsgEvent_t *pMsg );
static void oadManagerErrorRsp( attErrorRsp_t *pRsp );
static void oadManagerSvcDiscovery( void );
static void oadManagerCCCDiscovery( void );
static void oadManagerCharDiscovery( void );
static void oadManagerDevDiscovery( void );
static bool oadManagerFindServUUID( uint8 *pData, uint8 dataLen );
static void oadManagerAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static void oadManagerEnableNoti( uint16 handle );
static void oadManagerHandleNoti(attHandleValueNoti_t *pNoti);
static void oadManagerSendImgNotify(void);
char *bdAddr2Str ( uint8 *pAddr );

#include "sbl_exec_uart.c"

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t oadManagerRoleCB =
{
  NULL,             // RSSI callback
  oadManagerEventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      OADManager_Init
 *
 * @brief   Initialization function for the OAD Manager App Task.
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
void OADManager_Init( uint8 task_id )
{
  oadManagerTaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }

  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) oadManagerDeviceName );

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( task_id );

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  // Register for all key events - This app will handle all key events
  RegisterForKeys( task_id );
#endif
#if (defined HAL_LED) && (HAL_LED == TRUE)
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
#endif

  // Setup a delayed profile startup
  osal_set_event( task_id, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      OADManager_ProcessEvent
 *
 * @brief   OAD Manager Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 OADManager_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( task_id )) != NULL )
    {
      oadManager_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      VOID osal_msg_deallocate( pMsg );
    }

    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & DEV_DISCOVERY_EVT )
  {
    oadManagerDevDiscovery();

    return ( events ^ DEV_DISCOVERY_EVT );
  }

  if ( events & SVC_DISCOVERY_EVT )
  {
    oadManagerSvcDiscovery();

    return ( events ^ SVC_DISCOVERY_EVT );
  }

  if ( events & CCC_DISCOVERY_EVT )
  {
    oadManagerCCCDiscovery();

    return ( events ^ CCC_DISCOVERY_EVT );
  }

  if ( events & CHAR_DISCOVERY_EVT )
  {
    oadManagerCharDiscovery();

    return ( events ^ CHAR_DISCOVERY_EVT );
  }

  if ( events & CONN_INTERVAL_EVT )
  {
    (void)osal_set_event(oadManagerTaskId, SVC_DISCOVERY_EVT);

    GAPCentralRole_UpdateLink( oadManagerConnHandle,
                               DEFAULT_OAD_MIN_CONN_INTERVAL,
                               DEFAULT_OAD_MAX_CONN_INTERVAL,
                               DEFAULT_OAD_SLAVE_LATENCY,
                               DEFAULT_OAD_CONN_TIMEOUT );

    return ( events ^ CONN_INTERVAL_EVT );
  }

  if ( events & START_DEVICE_EVT )
  {
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &oadManagerRoleCB );
    oadManagerDevDiscovery();

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & OAD_DOWNLOAD_EVT )
  {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    if ( (oadBlkNum + 1) >= oadBlkTot )
    {
      LCD_WRITE_STRING("OAD Completed!", HAL_LCD_LINE_3);
    }
    else
    {
      LCD_WRITE_STRING("OAD Failed!", HAL_LCD_LINE_3);
    }
#endif

    return ( events ^ OAD_DOWNLOAD_EVT );
  }

  return 0;
}

/*********************************************************************
 * @fn      oadManager_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void oadManager_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  case KEY_CHANGE:
    // Make sure the OAD download timer is not running
    if ( osal_get_timeoutEx( oadManagerTaskId, OAD_DOWNLOAD_EVT ) == 0 )
    {
      oadManager_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
    }
    break;
#endif

  case GATT_MSG_EVENT:
    oadManagerProcessGATTMsg( (gattMsgEvent_t *) pMsg );
    break;
  }
}

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
/*********************************************************************
 * @fn      oadManager_HandleKeys
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
static void oadManager_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )  // Start or stop discovery
  {
    if ( oadManagerState != BLE_STATE_CONNECTED )
    {
      if ( !oadManagerScanning )
      {
        (void)osal_set_event(oadManagerTaskId, DEV_DISCOVERY_EVT);
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( !oadManagerScanning && oadManagerScanRes > 0 )
    {
      // Increment index of current result (with wraparound)
      oadManagerScanIdx++;
      if ( oadManagerScanIdx >= oadManagerScanRes )
      {
        oadManagerScanIdx = 0;
      }

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      LCD_WRITE_STRING_VALUE( "Device", oadManagerScanIdx + 1, 10, HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( bdAddr2Str( oadManagerDevList[oadManagerScanIdx].addr ), HAL_LCD_LINE_2 );
#endif
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
  }

  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;

    // Connect or disconnect
    if ( oadManagerState == BLE_STATE_IDLE )
    {
      // if there is a scan result
      if ( oadManagerScanRes > 0 )
      {
        if ( oadManagerScanIdx < oadManagerScanRes )
        {
          // connect to current device in scan result
          peerAddr = oadManagerDevList[oadManagerScanIdx].addr;
          addrType = oadManagerDevList[oadManagerScanIdx].addrType;

          oadManagerState = BLE_STATE_CONNECTING;

          GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                        DEFAULT_LINK_WHITE_LIST,
                                        addrType, peerAddr );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 );
        }
        else
        {
          LCD_WRITE_STRING( "Select device", HAL_LCD_LINE_3 );
#endif
        }
      }
    }
    else if ( oadManagerState == BLE_STATE_CONNECTING || oadManagerState == BLE_STATE_CONNECTED )
    {
      // disconnect
      oadManagerState = BLE_STATE_DISCONNECTING;

      VOID GAPCentralRole_TerminateLink( oadManagerConnHandle );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
#endif
    }
  }

  if ( keys & HAL_KEY_DOWN )
  {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    LCD_WRITE_STRING("SBL Mode...", HAL_LCD_LINE_3);
#endif
    EA = 0;
    sblRun();
    EA = 1;
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    LCD_WRITE_STRING("SBL Done.", HAL_LCD_LINE_3);
#endif
  }
}
#endif

/*********************************************************************
 * @fn      oadManagerProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void oadManagerProcessGATTMsg( gattMsgEvent_t *pPkt )
{
  if (oadManagerState != BLE_STATE_CONNECTED)
  {
    return;  // In case a GATT message came after a connection has dropped, ignore the message.
  }

  switch (pPkt->method)
  {
    case ATT_FIND_BY_TYPE_VALUE_RSP:
      oadManagerSvcDiscoveryMsg( pPkt );
      break;

    case ATT_FIND_INFO_RSP:
      oadManagerCCCDiscoveryMsg( pPkt );
      break;

    case ATT_READ_BY_TYPE_RSP:
      oadManagerCharDiscoveryMsg( pPkt );
      break;

    case ATT_HANDLE_VALUE_NOTI:
    case ATT_HANDLE_VALUE_IND:
      if (pPkt->hdr.status == SUCCESS)
      {
        oadManagerHandleNoti(&(pPkt->msg.handleValueNoti));
      }
      break;

   case ATT_ERROR_RSP:
      oadManagerErrorRsp( &(pPkt->msg.errorRsp) );
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      oadManagerEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void oadManagerEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        VOID osal_memcpy( oadManagerAddr, pEvent->initDone.devAddr, B_ADDR_LEN );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING( "OAD Manager", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( oadManagerAddr ),  HAL_LCD_LINE_2 );
#endif
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( oadManagerFindServUUID( pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen ) )
          {
            oadManagerAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        oadManagerScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          oadManagerScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( oadManagerDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING_VALUE( "Devices Found", oadManagerScanRes, 10, HAL_LCD_LINE_1 );
        if ( oadManagerScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        }
#endif
        // initialize scan index to last device
        oadManagerScanIdx = oadManagerScanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {
          oadManagerState = BLE_STATE_CONNECTED;
          oadManagerConnHandle = pEvent->linkCmpl.connectionHandle;

          (void)osal_set_event(oadManagerTaskId, CONN_INTERVAL_EVT);

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
#endif
        }
        else
        {
          oadManagerState = BLE_STATE_IDLE;
          oadManagerConnHandle = GAP_CONNHANDLE_INIT;
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        oadManagerState = BLE_STATE_IDLE;
        oadManagerConnHandle = GAP_CONNHANDLE_INIT;

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING( "OAD Manager", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( oadManagerAddr ),  HAL_LCD_LINE_2 );
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_3 );
#endif
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      oadManagerSvcDiscovery
 *
 * @brief   OAD Service discovery.
 *
 * @return  none
 */
static void oadManagerSvcDiscovery(void)
{
  uint8 oadServUUID[ATT_UUID_SIZE] = { TI_BASE_UUID_128( OAD_SERVICE_UUID ) };

  // Initialize service discovery variables
  oadSvcStartHdl = oadSvcEndHdl = 0;

  if (GATT_DiscPrimaryServiceByUUID(oadManagerConnHandle, oadServUUID,
                                    ATT_UUID_SIZE, oadManagerTaskId) != SUCCESS)
  {
    (void)osal_set_event(oadManagerTaskId, SVC_DISCOVERY_EVT);
  }
}

/*********************************************************************
 * @fn      oadManagerCCCDiscovery
 *
 * @brief   OAD Client Characteristic Configuration discovery.
 *
 * @return  none
 */
static void oadManagerCCCDiscovery(void)
{
  // Initialize CCCD discovery variable
  oadManagerCCCIdx = 0;

  // Discover characteristic descriptors
  if (GATT_DiscAllCharDescs(oadManagerConnHandle, oadSvcStartHdl,
                            oadSvcEndHdl, oadManagerTaskId) != SUCCESS)
  {
    (void)osal_set_event(oadManagerTaskId, CCC_DISCOVERY_EVT);
  }
}

/*********************************************************************
 * @fn      oadManagerCharDiscovery
 *
 * @brief   OAD Characteristics service discovery.
 *
 * @return  none
 */
static void oadManagerCharDiscovery(void)
{
  attReadByTypeReq_t req;

  req.startHandle = oadSvcStartHdl;
  req.endHandle = oadSvcEndHdl;
  req.type.len = ATT_UUID_SIZE;

  if ( oadManagerDiscIdx == 0 )
  {
    uint8 oadCharUUID[ATT_UUID_SIZE] = { TI_BASE_UUID_128( OAD_IMG_IDENTIFY_UUID ) };

    (void)osal_memcpy(req.type.uuid, oadCharUUID, ATT_UUID_SIZE);
  }
  else
  {
    uint8 oadCharUUID[ATT_UUID_SIZE] = { TI_BASE_UUID_128( OAD_IMG_BLOCK_UUID ) };

    (void)osal_memcpy(req.type.uuid, oadCharUUID, ATT_UUID_SIZE);
  }

  if (GATT_DiscCharsByUUID(oadManagerConnHandle, &req, oadManagerTaskId) != SUCCESS)
  {
    (void)osal_set_event(oadManagerTaskId, CHAR_DISCOVERY_EVT);
  }
}

/*********************************************************************
 * @fn      oadManagerDevDiscovery
 *
 * @brief   OAD device discovery.
 *
 * @return  none
 */
static void oadManagerDevDiscovery(void)
{
  oadManagerScanning = TRUE;
  oadManagerScanRes = 0;

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
  LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
  LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
#endif

  GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                 DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                 DEFAULT_DISCOVERY_WHITE_LIST );
}

/*********************************************************************
 * @fn      oadManagerSvcDiscoveryMsg
 *
 * @brief   Process GATT Primary Service discovery message
 *
 * @return  none
 */
static void oadManagerSvcDiscoveryMsg(gattMsgEvent_t *pMsg)
{
  if ( pMsg->hdr.status == SUCCESS ) // Characteristic found, the store handle.
  {
    attFindByTypeValueRsp_t *pRsp = &(pMsg->msg.findByTypeValueRsp);

    if ( pRsp->numInfo > 0 )
    {
      oadSvcStartHdl = pRsp->handlesInfo[0].handle;
      oadSvcEndHdl = pRsp->handlesInfo[0].grpEndHandle;

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      LCD_WRITE_STRING("OAD Svc Found!", HAL_LCD_LINE_1);
#endif
      // OAD service found
      (void)osal_set_event(oadManagerTaskId, CCC_DISCOVERY_EVT);
    }
  }
  else if ( pMsg->hdr.status == bleProcedureComplete )
  {
    if ( oadSvcStartHdl == 0 )
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING("OAD SvcNotFound", HAL_LCD_LINE_3);
#endif
    }
  }
}

/*********************************************************************
 * @fn      oadManagerCCCDiscoveryMsg
 *
 * @brief   Process GATT Client Characteristic Configuration discovery message
 *
 * @return  none
 */
static void oadManagerCCCDiscoveryMsg(gattMsgEvent_t *pMsg)
{
  if (oadManagerCCCIdx < OAD_CHAR_CNT)
  {
    if ( pMsg->hdr.status == SUCCESS ) // CCCD found, the store handle.
    {
      attFindInfoRsp_t *pRsp = &(pMsg->msg.findInfoRsp);

      if ( ( pRsp->numInfo > 0 ) && ( pRsp->format == ATT_HANDLE_BT_UUID_TYPE ) )
      {
        // for each handle/uuid pair
        for ( uint8 i = 0; i < pRsp->numInfo; i++ )
        {
          // look for CCCDs
          if ( ( pRsp->info.btPair[i].uuid[0] == LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID) ) &&
               ( pRsp->info.btPair[i].uuid[1] == HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID) ) )
          {
            // CCCD found
            oadManagerCCCHandles[oadManagerCCCIdx] = pRsp->info.btPair[i].handle;

            if (++oadManagerCCCIdx == OAD_CHAR_CNT)
            {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
              LCD_WRITE_STRING("OAD CCCDs Found!", HAL_LCD_LINE_1);
#endif
              // OAD CCCDs found; enable them for notification
              oadManagerEnableNoti( oadManagerCCCHandles[OAD_CHAR_IMG_IDENTIFY] );
              oadManagerEnableNoti( oadManagerCCCHandles[OAD_CHAR_IMG_BLOCK] );

              oadManagerDiscIdx = 0;
              (void)osal_set_event(oadManagerTaskId, CHAR_DISCOVERY_EVT);

              break;
            }
          }
        } // for
      }
    }
    else if ( pMsg->hdr.status == bleProcedureComplete )
    {
      if (oadManagerCCCIdx < OAD_CHAR_CNT)
      {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING("OAD CCCDNotFound", HAL_LCD_LINE_3);
#endif
      }
    }
  }
}

/*********************************************************************
 * @fn      oadManagerCharDiscoveryMsg
 *
 * @brief   Process GATT Characteristic discovery message
 *
 * @return  none
 */
static void oadManagerCharDiscoveryMsg(gattMsgEvent_t *pMsg)
{
  if (oadManagerDiscIdx < OAD_CHAR_CNT)
  {
    if ( pMsg->hdr.status == SUCCESS ) // Characteristic found, the store handle.
    {
      attReadByTypeRsp_t *pRsp = &(pMsg->msg.readByTypeRsp);

      if (pRsp->numPairs > 0)
      {
        oadManagerHandles[oadManagerDiscIdx] = BUILD_UINT16(pRsp->dataList[3], pRsp->dataList[4]);

        if (++oadManagerDiscIdx == OAD_CHAR_CNT)
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING("OAD Chars Found!", HAL_LCD_LINE_1);
#endif
          oadManagerSendImgNotify();

          return;
        }
      }

      (void)osal_set_event(oadManagerTaskId, CHAR_DISCOVERY_EVT);
    }
    else if ( pMsg->hdr.status == bleProcedureComplete )
    {
      if (oadManagerDiscIdx < OAD_CHAR_CNT)
      {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        LCD_WRITE_STRING("OAD CharNotFound", HAL_LCD_LINE_3);
#endif
      }
    }
  }
}

/*********************************************************************
 * @fn      oadManagerErrorRsp
 *
 * @brief   Process GATT Error Response message
 *
 * @return  none
 */
static void oadManagerErrorRsp( attErrorRsp_t *pRsp )
{
  if ( pRsp->errCode == ATT_ERR_ATTR_NOT_FOUND )
  {
    switch ( pRsp->reqOpcode )
    {
      case ATT_FIND_BY_TYPE_VALUE_REQ:
        if ( oadSvcStartHdl == 0 )
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING("OAD SvcNotFound", HAL_LCD_LINE_3);
#endif
        }
        break;

      case ATT_FIND_INFO_RSP:
        if (oadManagerCCCIdx < OAD_CHAR_CNT)
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING("OAD CCCDNotFound", HAL_LCD_LINE_3);
#endif
        }
        break;

      case ATT_READ_BY_TYPE_RSP:
        if (oadManagerDiscIdx < OAD_CHAR_CNT)
        {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          LCD_WRITE_STRING("OAD CharNotFound", HAL_LCD_LINE_3);
#endif
        }
        break;

      default:
        break;
    }
  }
}

/*********************************************************************
 * @fn      oadManagerFindServUUID
 *
 * @brief   Find the OAD Service UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool oadManagerFindServUUID( uint8 *pData, uint8 dataLen )
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

      if ( adType == GAP_ADTYPE_128BIT_MORE || adType == GAP_ADTYPE_128BIT_COMPLETE )
      {
        pData++;
        adLen--;

        while ((adLen >= ATT_UUID_SIZE) && (pData < pEnd))     // For each UUID in list.
        {
          uint8 oadServUUID[ATT_UUID_SIZE] = { TI_BASE_UUID_128( OAD_SERVICE_UUID ) };

          if (osal_memcmp(pData, oadServUUID, ATT_UUID_SIZE))  // Check for match.
          {
            return TRUE;  // Match found
          }

          // Go to next
          pData += ATT_UUID_SIZE;
          adLen -= ATT_UUID_SIZE;
        }

        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else  // Go to next item
      {
        pData += adLen;
      }
    }
  }

  return FALSE;  // Match not found
}

/*********************************************************************
 * @fn      oadManagerAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void oadManagerAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;

  // If result count not at max
  if ( oadManagerScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < oadManagerScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, oadManagerDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }

    // Add addr to scan result list
    osal_memcpy( oadManagerDevList[oadManagerScanRes].addr, pAddr, B_ADDR_LEN );
    oadManagerDevList[oadManagerScanRes].addrType = addrType;

    // Increment scan result count
    oadManagerScanRes++;
  }
}

/*********************************************************************
 * @fn      oadManagerEnableNoti
 *
 * @brief   Enable notification on the OAD Target
 *
 * @return  none
 */
static void oadManagerEnableNoti( uint16 handle )
{
  attWriteReq_t req;

  req.handle = handle;
  req.len = 2;
  req.sig = FALSE;
  req.cmd = TRUE;

  req.value[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
  req.value[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);

  VOID GATT_WriteNoRsp(oadManagerConnHandle, &req);
}

/*********************************************************************
 * @fn      oadManagerHandleNoti
 *
 * @brief   Handle Notifications and Indications.
 *
 * @return  none
 */
static void oadManagerHandleNoti(attHandleValueNoti_t *pNoti)
{
  if (pNoti->handle == oadManagerHandles[OAD_CHAR_IMG_IDENTIFY])
  {
#if (defined HAL_LCD && (HAL_LCD == TRUE))
    uint16 ver = BUILD_UINT16(pNoti->value[0], pNoti->value[1]);
    uint8 userId[12] = "UserId ";

    osal_memcpy(&(userId[7]), &(pNoti->value[4]), 4);
    userId[11] = '\0';

    if ( OAD_IMG_ID( ver ) == 0 )
    {
        HalLcdWriteStringValueValue("ImgA Id", OAD_VER_NUM( ver ), 16,
                                               BUILD_UINT16(pNoti->value[2], pNoti->value[3]), 16,
                                               HAL_LCD_LINE_2);
    }
    else
    {
        HalLcdWriteStringValueValue("ImgB Id", OAD_VER_NUM( ver ), 16,
                                               BUILD_UINT16(pNoti->value[2], pNoti->value[3]), 16,
                                               HAL_LCD_LINE_2);
    }

    HalLcdWriteString((char*)userId, HAL_LCD_LINE_3);
#endif
  }
  else if (pNoti->handle == oadManagerHandles[OAD_CHAR_IMG_BLOCK])
  {
    oadBlkNum = BUILD_UINT16(pNoti->value[0], pNoti->value[1]);
    attWriteReq_t req;

    req.handle = oadManagerHandles[OAD_CHAR_IMG_BLOCK];
    req.len = 2 + OAD_BLOCK_SIZE;
    req.sig = FALSE;
    req.cmd = TRUE;

    req.value[0] = LO_UINT16(oadBlkNum);
    req.value[1] = HI_UINT16(oadBlkNum);

    uint8 page = oadBlkNum / OAD_BLOCKS_PER_PAGE;
    uint16 oset = (oadBlkNum - (OAD_BLOCKS_PER_PAGE * page)) * OAD_BLOCK_SIZE;
    HalFlashRead(page+OAD_IMG_B_PAGE, oset, req.value+2, OAD_BLOCK_SIZE);

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
    if (oadBlkNum == 0)
    {
      HalLcdWriteString("", HAL_LCD_LINE_3);
    }
    HalLcdDisplayPercentBar( "OAD Progress...", (oadBlkNum / (oadBlkTot / 100)) );
#endif

    VOID GATT_WriteNoRsp(oadManagerConnHandle, &req);

    VOID osal_start_timerEx( oadManagerTaskId, OAD_DOWNLOAD_EVT, OAD_DOWNLOAD_TIMEOUT );
  }
}

/*********************************************************************
 * @fn      oadManagerSendImgNotify
 *
 * @brief   OAD device discovery.
 *
 * @return  none
 */
static void oadManagerSendImgNotify(void)
{
  attWriteReq_t req;
  img_hdr_t ImgHdr;

  HalFlashRead(OAD_IMG_B_PAGE, OAD_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(img_hdr_t));

  req.handle = oadManagerHandles[OAD_CHAR_IMG_IDENTIFY];
  req.len = OAD_IMG_HDR_SIZE;
  req.value[0] = LO_UINT16(ImgHdr.ver);
  req.value[1] = HI_UINT16(ImgHdr.ver);

  req.value[2] = LO_UINT16(ImgHdr.len);
  req.value[3] = HI_UINT16(ImgHdr.len);

  (void)osal_memcpy(req.value+4, ImgHdr.uid, sizeof(ImgHdr.uid));

  req.sig = FALSE;
  req.cmd = TRUE;

  VOID GATT_WriteNoRsp(oadManagerConnHandle, &req);

  // Save the total number of blocks
  oadBlkTot = ImgHdr.len / (OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);
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
