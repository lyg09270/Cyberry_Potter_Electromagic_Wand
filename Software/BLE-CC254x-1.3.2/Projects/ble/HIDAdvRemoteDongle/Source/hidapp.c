/**************************************************************************************************
  Filename:       hidapp.c
  Revised:        $Date: 2013-03-18 13:45:07 -0700 (Mon, 18 Mar 2013) $
  Revision:       $Revision: 33513 $

  Description:    This file contains the BLE sample application for HID dongle
                  application.

  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "bcomdef.h"
#include "linkdb.h"

// HAL includes
#include "hal_types.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_key.h"
#include "hal_led.h"

// HID includes
#include "usb_framework.h"
#include "usb_hid.h"
#include "usb_hid_reports.h"
#include "usb_suspend.h"

// OSAL includes
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "osal_bufmgr.h"
#include "OSAL_Tasks.h"
#include "osal_snv.h"

#include "OnBoard.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbTimer.h"
#endif

/* GATT */
#include "gatt.h"
#include "gatt_uuid.h"

/* HCI */
#include "hci.h"

#include "central.h"
#include "gapbondmgr.h"
#include "gattservapp.h"

/* HID */
#include "hid_uuid.h"

/* Application */
#include "hidapp.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#define OWN_ADDR                              { 0x11, 0x11, 0x11, 0x11, 0x11, 0x04 }

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_LIMITED

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 900

// Initiate connection timer for application
#define INIT_CONNECT_TIMEOUT                  1000

// Number of scans when device not bonded
#define MAX_NUM_SCANS                         5

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE // GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

#define DEFAULT_MAX_SCAN_RES                  8

#define APP_CONN_INTERVAL                     8      // 10ms
#define APP_SLAVE_LATENCY                     0      // Initially 0 for fast connection. //49     // 49 slave latency (500ms effective interval)
#define APP_CONN_TIMEOUT                      500    // 1.6s supervision timeout

// output report buffer size
#define HIDAPP_OUTBUF_SIZE                    3

// Vendor specific report identifiers
#define HID_CMD_REPORT_ID                     1
#define HID_PAIR_ENTRY_REPORT_ID              2

// output report ID
#define HIDAPP_OUTPUT_REPORT_ID               HID_CMD_REPORT_ID

// Count of polling for INPUT packet ready
#define USB_HID_INPUT_RETRY_COUNT             3

#define HIDAPP_INPUT_RETRY_TIMEOUT            5 // ms

// Service Change flags
#define NO_CHANGE                             0x00
#define CHANGE_OCCURED                        0x01

// Gap Bond Manager States
#define UNPAIRED_STATE                        0x00
#define PAIRED_BONDED_STATE                   0x01

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef struct
{
  // Service and Characteristic discovery variables.
  uint16 mouseCharHandle;
  uint16 keyCharHandle;
  uint16 consumerCtrlCharHandle;

  // CCC's of the notifications
  uint16 mouseCCCHandle;
  uint16 keyCCCHandle;
  uint16 consumerCtrlCCCHandle;
  uint16 svcChangeHandle;
  uint8  lastRemoteAddr[B_ADDR_LEN];
} hidappHandleInfo_t;

// enumerated type for current central BLE
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_SCANNING,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
static void hidappStart( void );
//static void hidappOutputReport( void );
static uint8 hidappSendInReport( attHandleValueNoti_t *pNoti );
static void hidappEstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr );
static void hidappStartDiscovery( void );
static void hidappDiscoverDevices( void );
static void hidappSetIdle( void );
static void hidappProcessGATTMsg( gattMsgEvent_t *pMsg );
static void hidappHandleKeys( uint8 keys, uint8 state );
static void hidappEnableNotification( uint16 connHandle, uint16 attrHandle );
static void hidappDiscoverService( uint16 connHandle, uint16 svcUuid );
static bool hidappFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static uint8 hidappFindHIDRemote( uint8* pData, uint8 length );
static void hidappSaveHandles( void );
static void hidappEraseHandles( void );
static uint8 hidappBondCount( void );

// Callback functions
static void hidappCentralEventCB( gapCentralRoleEvent_t *p );
static void hidappPairStateCB( uint16 connHandle, uint8 state, uint8 status );

void hidappSuspendEnter( void );
void hidappSuspendExit( void );

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

//// HID output report buffer
//static uint8 hidappOutBuf[HIDAPP_OUTBUF_SIZE];

// HID input report used for retries
static attHandleValueNoti_t lastInReport;
static uint8 reportRetries = 0;

// OSAL task ID assigned to the application task
static uint8 hidappTaskId;

// GAP Role Callbacks
static gapCentralRoleCB_t hidApp_centralCBs =
{
  NULL,                 // When a valid RSSI is read from controllerNULL,
  hidappCentralEventCB, // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
gapBondCBs_t gapBondCBs =
{
  NULL,                 // Passcode callback
  hidappPairStateCB,    // Pairing state callback
};

// Application state
static uint8 hidappBLEState = BLE_STATE_IDLE;

static uint16 connHandle = INVALID_CONNHANDLE;

// Service and Characteristic discovery variables.
static uint16 mouseCharHandle        = GATT_INVALID_HANDLE;
static uint16 keyCharHandle          = GATT_INVALID_HANDLE;
static uint16 consumerCtrlCharHandle = GATT_INVALID_HANDLE;

// CCC's of the notifications
static uint16 mouseCCCHandle         = GATT_INVALID_HANDLE;
static uint16 keyCCCHandle           = GATT_INVALID_HANDLE;
static uint16 consumerCtrlCCCHandle  = GATT_INVALID_HANDLE;

// Service Change Handle
static uint16 serviceChangeHandle = GATT_INVALID_HANDLE;

static uint8 serviceDiscComplete = FALSE;

static uint8 remoteAddr[B_ADDR_LEN] = {0,0,0,0,0,0};

// Handle info saved here after connection to skip service discovery.
static hidappHandleInfo_t remoteHandles;

static uint8 serviceChange = NO_CHANGE;

static uint8 gapBondMgrState = UNPAIRED_STATE;

static uint16 serviceToDiscover = GATT_INVALID_HANDLE;

static uint8 enableCCCDs = TRUE;

// Variables used for service/attribute discovery
static attReadByTypeReq_t readReq;
static attAttrType_t readReqType;
static uint16 serviceStartHandle;
static uint16 serviceEndHandle;

static uint8 numScans = 0;

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/*******************************************************************************
 *
 * @fn      hidappInit
 *
 * @brief   This is the Sample Application task initialization called by OSAL.
 *
 * @param   taskId - task ID assigned after it was added in the OSAL task queue
 *
 * @return  none
 */
void Hidapp_Init( uint8 taskId )
{
  // save task ID assigned by OSAL
  hidappTaskId = taskId;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    VOID GAPCentralRole_SetParameter( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }

  // Setup GAP
  {
    VOID GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    VOID GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

    // Set scanning interval, 48 = 30ms
    VOID GAP_SetParamValue( TGAP_LIM_DISC_SCAN_INT, 48 );

    // Set scannng window, 48 = 30ms
    VOID GAP_SetParamValue( TGAP_LIM_DISC_SCAN_WIND, 48 );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    uint8 autoSync = TRUE;
    uint8 bondFailAction = GAPBOND_FAIL_TERMINATE_ERASE_BONDS;

    VOID GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    VOID GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    VOID GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    VOID GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    VOID GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
    VOID GAPBondMgr_SetParameter( GAPBOND_AUTO_SYNC_WL, sizeof( uint8 ), &autoSync );
    VOID GAPBondMgr_SetParameter( GAPBOND_BOND_FAIL_ACTION, sizeof( uint8 ), &bondFailAction );
  }

  // USB suspend entry/exit hook function setup
  pFnSuspendEnterHook = hidappSuspendEnter;
  pFnSuspendExitHook = hidappSuspendExit;

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive notifications
  GATT_RegisterForInd( hidappTaskId );

  // Set connection parameters:
  {
    uint16 connectionInterval = APP_CONN_INTERVAL;
    uint16 slaveLatency = APP_SLAVE_LATENCY;
    uint16 timeout = APP_CONN_TIMEOUT;

    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, connectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, connectionInterval );
    VOID GAP_SetParamValue( TGAP_CONN_EST_LATENCY, slaveLatency );
    VOID GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, timeout );
  }

  // Register callbacks with the GAP Bond Manager.
  GAPBondMgr_Register( &gapBondCBs );

  //HalKeyConfig(HAL_KEY_INTERRUPT_ENABLE, hidappHandleKeys);
  RegisterForKeys( hidappTaskId );

  // set up continued initialization from within OSAL task loop
  VOID osal_start_timerEx( taskId, HIDAPP_EVT_START, 100 );

  // Clear remote data
  hidappEraseHandles();

  VOID HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE );
}

/*********************************************************************
 * @fn      Hidapp_ProcessEvent
 *
 * @brief   This function processes the OSAL events and messages for the
 *          Sample Application.
 *
 * input parameters
 *
 * @param   taskId - Task ID assigned to this app by OSAL at system initialization.
 * @param   events - Bit mask of the pending event(s).
 *
 * output parameters
 *
 * None.
 *
 * @return  The events bit map received via parameter with the bits cleared
 *          which correspond to the event(s) that were processed on this invocation.
 */
uint16 Hidapp_ProcessEvent(uint8 taskId, uint16 events)
{
  (void) taskId; // Unused argument

  if ( events & SYS_EVENT_MSG )
  {
    osal_event_hdr_t  *pMsg;

    while ((pMsg = (osal_event_hdr_t *) osal_msg_receive(hidappTaskId)) != NULL)
    {
      switch (pMsg->event)
      {
        case GATT_MSG_EVENT:
          {
            hidappProcessGATTMsg( (gattMsgEvent_t *)pMsg );
          }
          break;
        case KEY_CHANGE:
          {
            hidappHandleKeys( ((keyChange_t *)pMsg)->keys, ((keyChange_t *)pMsg)->state );
          }
          break;

        default:
          break;
      }

      VOID osal_msg_deallocate((uint8 *) pMsg);
    }

    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & HIDAPP_EVT_START )
  {
    hidappStart();

    return (events ^ HIDAPP_EVT_START);
  }

  if ( events & HIDAPP_EVT_REPORT_RETRY )
  {
    // Report retries event
    if ( ( hidappSendInReport( &lastInReport ) == FALSE ) &&
         ( reportRetries < USB_HID_INPUT_RETRY_COUNT ) )
    {
      reportRetries++;
      osal_start_timerEx( hidappTaskId, HIDAPP_EVT_REPORT_RETRY,
                          (reportRetries*HIDAPP_INPUT_RETRY_TIMEOUT) );
    }
    else
    {
      // Done retrying
      reportRetries = 0;
    }

    return (events ^ HIDAPP_EVT_REPORT_RETRY);
  }

  if ( events & HIDAPP_EVT_START_DISCOVERY )
  {
    // Start initial discovery
    hidappStartDiscovery();

    return ( events ^ HIDAPP_EVT_START_DISCOVERY );
  }

  if ( events & HIDAPP_EVT_INIT_CONNECT )
  {
    // Still trying to re-connect
    HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK ); // green led

    return ( events ^ HIDAPP_EVT_INIT_CONNECT );
  }

  return ( 0 );  /* Discard unknown events. */
}

/*********************************************************************
 *
 * @fn      hidappHandleKey
 *
 * @brief   Handle service for keys
 *
 * @param   keys  - key that was pressed (i.e. the scanned row/col index)
 *          state - shifted
 *
 * @return  void
 */
static void hidappHandleKeys( uint8 keys, uint8 state )
{
  // Unused arguments
  (void) state;

  if (keys & HAL_KEY_SW_1)
  {
    // If bonds exist, erase all of them
    if ( ( hidappBondCount() > 0 ) && ( hidappBLEState != BLE_STATE_CONNECTED ) )
    {
      if ( hidappBLEState == BLE_STATE_CONNECTING )
      {
        hidappBLEState = BLE_STATE_DISCONNECTING;
        VOID GAPCentralRole_TerminateLink( GAP_CONNHANDLE_INIT );
      }

      VOID GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
    }
  }

  if (keys & HAL_KEY_SW_2)
  {
    if ( hidappBLEState == BLE_STATE_CONNECTED )
    {
      hidappBLEState = BLE_STATE_DISCONNECTING;
      VOID GAPCentralRole_TerminateLink( connHandle );
    }
    else if ( hidappBLEState == BLE_STATE_IDLE )
    {
      #if defined ( NANO_DONGLE )

      HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF ); // red led

      // Notify our task to start initial discovey
      osal_set_event( hidappTaskId, HIDAPP_EVT_START_DISCOVERY );

      #endif //  #if defined ( NANO_DONGLE )
    }
  }
}

/*********************************************************************
 *
 * @fn      hidappStart
 *
 * @brief   Start up the application
 *
 * @param   None
 *
 * @return  None
 */
static void hidappStart(void)
{
  GAPCentralRole_StartDevice( &hidApp_centralCBs );
}

///*********************************************************************
// *
// * @fn      hidappOutputReport
// *
// * @brief   Handles output report from host
// *
// * @param   None
// *
// * @return  None
// */
//static void hidappOutputReport( void )
//{
//  // Decode the output report
//  if (HIDAPP_OUTPUT_REPORT_ID == hidappOutBuf[0])
//  {
//    // correct report ID
//    uint8 cmdId = hidappOutBuf[1];
//    switch (cmdId)
//    {
//      // Process output buffer here
//    }
//  }
//}

/*********************************************************************
 *
 * @fn      hidappSuspendEnter
 *
 * @brief   Hook function to be called upon entry into USB suspend mode
 *
 * @param   none
 *
 * @return  none
 */
void hidappSuspendEnter( void )
{
  // not supported
}

/*********************************************************************
 *
 * @fn      hidappSuspendExit
 *
 * @brief   Hook function to be called upon exit from USB suspend mode
 *
 * @param   none
 *
 * @return  none
 */
void hidappSuspendExit( void )
{
  // not supported
}

/*********************************************************************
 * @fn      hidappProcessGATTMsg
 *
 * @brief   Process incoming GATT messages.
 *
 * @param   pPkt - pointer to message.
 *
 * @return  none
 */

static void hidappProcessGATTMsg( gattMsgEvent_t *pPkt )
{
  // Build the message first
  switch ( pPkt->method )
  {
    case ATT_HANDLE_VALUE_NOTI:
      // First try to send out pending HID report
      if ( reportRetries > 0 )
      {
        hidappSendInReport( &lastInReport );

        reportRetries = 0;
        osal_stop_timerEx( hidappTaskId, HIDAPP_EVT_REPORT_RETRY );
      }

      // Send incoming HID report
      if ( hidappSendInReport( &(pPkt->msg.handleValueNoti) ) == FALSE )
      {
        // Save report for retries later
        osal_memcpy( &lastInReport, &(pPkt->msg.handleValueNoti), sizeof( attHandleValueNoti_t ) );

        reportRetries = 1;
        osal_start_timerEx( hidappTaskId, HIDAPP_EVT_REPORT_RETRY, HIDAPP_INPUT_RETRY_TIMEOUT );
      }
      break;

    case ATT_FIND_BY_TYPE_VALUE_RSP:
      // Response from GATT_DiscPrimaryServiceByUUID
      // Service found, store handles
      if ( pPkt->msg.findByTypeValueRsp.numInfo > 0 )
      {
        serviceStartHandle = pPkt->msg.findByTypeValueRsp.handlesInfo[0].handle;
        serviceEndHandle   = pPkt->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      // If procedure complete
      else if ( pPkt->hdr.status == bleProcedureComplete )
      {
        if ( serviceStartHandle != 0 )
        {
          if ( serviceToDiscover == GATT_SERVICE_UUID )
          {
            // Begin the search for characteristic handle of the service
            readReq.startHandle = serviceStartHandle;
            readReq.endHandle = serviceEndHandle;
            readReqType.len = 2;
            readReqType.uuid[0] = LO_UINT16(SERVICE_CHANGED_UUID);
            readReqType.uuid[1] = HI_UINT16(SERVICE_CHANGED_UUID);
            readReq.type = readReqType;

            GATT_DiscCharsByUUID(connHandle, &readReq, hidappTaskId );
          }
          else if ( serviceToDiscover == HID_SERVICE_UUID )
          {
            // Discover all characteristics
            GATT_DiscAllChars( connHandle, serviceStartHandle, serviceEndHandle, hidappTaskId );
          }
        }
      }
      break;

    case ATT_READ_BY_TYPE_RSP:
      // Response from Discover all Characteristics.
      // Success indicates packet with characteristic discoveries.
      if ( pPkt->hdr.status == SUCCESS )
      {
        attReadByTypeRsp_t rsp = pPkt->msg.readByTypeRsp;
        uint8 idx = 0;

        if ( serviceToDiscover ==  GATT_SERVICE_UUID )
        {
          // We have discovered the GATT Service Characteristic handle
          uint8 low = LO_UINT16(pPkt->msg.readByTypeRsp.dataList[3]);
          uint8 high = HI_UINT16(pPkt->msg.readByTypeRsp.dataList[4]);
          serviceChangeHandle = BUILD_UINT16(low, high);
          // Break to skip next part, it doesn't apply here
          break;
        }

        // Search characteristics for those with notification permissions.
        while( idx < ((rsp.numPairs * rsp.len ) - 1))
        {
          // Check permissions of characteristic for notification permission
          if ( (rsp.dataList[idx+2] & GATT_PROP_NOTIFY ) )
          {
            uint16* pHandle = (mouseCharHandle == GATT_INVALID_HANDLE) ? &mouseCharHandle : &keyCharHandle ;

            if ( pHandle == &keyCharHandle && consumerCtrlCharHandle == GATT_INVALID_HANDLE )
            {
              pHandle = ( keyCharHandle == GATT_INVALID_HANDLE ) ? &keyCharHandle : &consumerCtrlCharHandle;
            }

            if ( *pHandle == GATT_INVALID_HANDLE )
            {
              *pHandle = BUILD_UINT16( rsp.dataList[idx+3], rsp.dataList[idx+4] );
            }
          }

          idx += rsp.len;
        }
      }
      // This indicates that there is no more characteristic data
      // to be discovered within the given handle range.
      else if ( pPkt->hdr.status == bleProcedureComplete )
      {
        if ( serviceToDiscover == GATT_SERVICE_UUID )
        {
          // Begin Service Discovery of HID Service
          serviceToDiscover = HID_SERVICE_UUID;
          hidappDiscoverService( connHandle, HID_SERVICE_UUID );
          // Break to skip next part, it doesn't apply yet.
          break;
        }

        if ( enableCCCDs == TRUE )
        {
          mouseCCCHandle = mouseCharHandle + 1;

          // Begin configuring the characteristics for notifications
          hidappEnableNotification( connHandle, mouseCCCHandle );
        }
        else
        {
          serviceDiscComplete = TRUE;
        }
      }
      break;

    case ATT_WRITE_RSP:
      if ( pPkt->hdr.status == SUCCESS && !serviceDiscComplete )
      {
        uint16 handle = ( keyCCCHandle == GATT_INVALID_HANDLE ) ?
                        ( keyCCCHandle = keyCharHandle + 1 ) :
                        ( consumerCtrlCCCHandle = consumerCtrlCharHandle + 1 );

        hidappEnableNotification( connHandle, handle );

        if ( consumerCtrlCCCHandle != GATT_INVALID_HANDLE)
        {
          serviceDiscComplete = TRUE;
        }
      }
      break;

    // Service Change indication
    case ATT_HANDLE_VALUE_IND:
      // Note: this logic assumes that the only indications that will be sent
      //       will come from that GATT Service Changed Characteristic
      if ( pPkt->hdr.status == SUCCESS )
      {
        serviceChange = CHANGE_OCCURED;

        // Acknowledge receipt of indication
        ATT_HandleValueCfm( pPkt->connHandle );

        // Handles in server have changed while devices are connected
        if ( ( gapBondMgrState == PAIRED_BONDED_STATE ) &&
            ( serviceChangeHandle == pPkt->msg.handleValueInd.handle ) )
        {
          // Begin Service Discovery of HID Service
          serviceToDiscover = HID_SERVICE_UUID;
          hidappDiscoverService( connHandle, HID_SERVICE_UUID );

          serviceChange = NO_CHANGE;
        }
      }
      break;

    default:
      // Unknown event
      break;
  }
}

/*********************************************************************
 * @fn      hidappSendInReport
 *
 * @brief   Send out an incoming HID report (GATT indication).
 *
 * @param   pNoti - report to be sent
 *
 * @return  TRUE if report was sent; FALSE otherwise.
 */
static uint8 hidappSendInReport( attHandleValueNoti_t *pNoti )
{
  uint8 endPoint;

  if( pNoti->handle == keyCharHandle )
  {
    // Keyboard report
    endPoint = USB_HID_KBD_EP;
  }
  else if (pNoti->handle == mouseCharHandle )
  {
    // Mouse report
    endPoint = USB_HID_MOUSE_EP;
  }
  else if ( pNoti->handle == consumerCtrlCharHandle )
  {
    // Consumer Control report
    endPoint = USB_HID_CC_EP;
  }
  else
  {
    // Maybe we're still in discovery phase
    return ( FALSE );
  }

  HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );

  return ( hidSendHidInReport(pNoti->value, endPoint, pNoti->len) );
}

/*********************************************************************
 * @fn      hidappCentralEventCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   pEvent - new role event
 *
 * @return  none
 */
static void hidappCentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  static uint8 addrType;
  static uint8 peerDeviceFound = FALSE;

  switch( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        gapDeviceInitDoneEvent_t *pEvt = (gapDeviceInitDoneEvent_t *)pEvent;

        // See if device has a valid BD Address
        if ( osal_isbufset( pEvt->devAddr, 0xFF, B_ADDR_LEN ) == FALSE )
        {
          #if defined ( NANO_DONGLE )

          if ( hidappBondCount() > 0 )
          {
            // Initiate connection
            hidappEstablishLink( TRUE, addrType, remoteAddr );
          }
          else
          {
            // Sit idle till ask to scan
            hidappSetIdle();
          }

          #endif //  #if defined ( NANO_DONGLE )
        }
        else
        {
          uint8 ownBDaddr[] = OWN_ADDR;

          // An old CC2540 USB Dongle has all 0xFF's for its BD Address so
          // just use the hard-coded address in that case.
          HCI_EXT_SetBDADDRCmd( ownBDaddr );

          // Re-initialize the device with the new address
          osal_set_event( hidappTaskId, HIDAPP_EVT_START );
        }
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      // Goes here before entering GAP_DEVICE_DISCOVERY_EVENT

      // Read advertising/scan response data
      // if it contains HID remote, end service discovery
      if ( hidappFindSvcUuid( HID_SERVICE_UUID,
                              pEvent->deviceInfo.pEvtData,
                              pEvent->deviceInfo.dataLen ) )
      {
        // Record peer devices address data
        addrType = pEvent->deviceInfo.addrType;
        osal_memcpy( remoteAddr, pEvent->deviceInfo.addr, B_ADDR_LEN );

        peerDeviceFound = TRUE;
      }

      if ( ( peerDeviceFound == TRUE ) &&
           ( pEvent->deviceInfo.eventType == GAP_ADTYPE_SCAN_RSP_IND ) &&
           hidappFindHIDRemote( pEvent->deviceInfo.pEvtData,
                                pEvent->deviceInfo.dataLen ) )
      {
        // End device discovery
        VOID GAPCentralRole_CancelDiscovery();
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      // If we have found a connectable device, establish a connection
      if ( peerDeviceFound == TRUE )
      {
        hidappEstablishLink( FALSE, addrType, remoteAddr );

        peerDeviceFound = FALSE;
        numScans = 0;
      }
      else if ( numScans > 0 )
      {
        numScans--;

        // Scan again
        hidappDiscoverDevices();
      }
      else
      {
        // Go idle
        hidappSetIdle();
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      // Cancel initiate connection timer
      VOID osal_stop_timerEx( hidappTaskId, HIDAPP_EVT_INIT_CONNECT );

      if ( pEvent->gap.hdr.status == SUCCESS )
      {
        hidappBLEState = BLE_STATE_CONNECTED;

        HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );  // green led

        connHandle = ((gapEstLinkReqEvent_t*)pEvent)->connectionHandle;
      }
      else if ( hidappBondCount() > 0 )
      {
        // Re-initiate connection
        hidappEstablishLink( TRUE, addrType, remoteAddr );
      }
      else
      {
        // Go idle
        hidappSetIdle();
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      // Cancel initiate connection timer
      VOID osal_stop_timerEx( hidappTaskId, HIDAPP_EVT_INIT_CONNECT );

      HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF ); // green led

      hidappBLEState = BLE_STATE_IDLE;
      gapBondMgrState = UNPAIRED_STATE;
      connHandle = INVALID_CONNHANDLE;

      if ( serviceDiscComplete == TRUE )
      {
        // Remember the address of the last connected remote
        osal_memcpy( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN );

        // Save handle information
        hidappSaveHandles();
      }

      // Invalidate service discovery variables.
      serviceDiscComplete    = FALSE;
      mouseCharHandle        = GATT_INVALID_HANDLE;
      keyCharHandle          = GATT_INVALID_HANDLE;
      consumerCtrlCharHandle = GATT_INVALID_HANDLE;

      mouseCCCHandle         = GATT_INVALID_HANDLE;
      keyCCCHandle           = GATT_INVALID_HANDLE;
      consumerCtrlCCCHandle  = GATT_INVALID_HANDLE;
      serviceChangeHandle    = GATT_INVALID_HANDLE;
      serviceToDiscover      = GATT_INVALID_HANDLE;

      enableCCCDs = TRUE;

      if ( hidappBondCount() > 0 )
      {
        // Re-initiate connection
        hidappEstablishLink( TRUE, addrType, remoteAddr );
      }
      else
      {
        // Go idle
        hidappSetIdle();
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      hidappEstablishLink
 *
 * @brief   Establish a link to a peer device.
 *
 * @param   whiteList - determines use of the white list
 * @param   addrType - address type of the peer devic
 * @param   remoteAddr - peer device address
 *
 * @return  none
 */
static void hidappEstablishLink( uint8 whiteList, uint8 addrType, uint8 *remoteAddr )
{
  if ( hidappBLEState != BLE_STATE_CONNECTED )
  {
    hidappBLEState = BLE_STATE_CONNECTING;

    // Try to connect to remote device
    VOID GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                       whiteList, addrType, remoteAddr );

    VOID osal_start_reload_timer( hidappTaskId, HIDAPP_EVT_INIT_CONNECT,
                                  (uint32)INIT_CONNECT_TIMEOUT );
  }
}

/*********************************************************************
 * @fn      hidappStartDiscovery
 *
 * @brief   Start initial scanning.
 *
 * @param   none
 *
 * @return  none
 */
static void hidappStartDiscovery( void )
{
  // If idle and not in a connection, we should start scanning
  if ( hidappBLEState != BLE_STATE_CONNECTED )
  {
    hidappBLEState = BLE_STATE_SCANNING;

    numScans = MAX_NUM_SCANS;

    // Begin initial scanning
    hidappDiscoverDevices();
  }
}

/*********************************************************************
 * @fn      hidappDiscoverDevices
 *
 * @brief   Start device discovery scan
 *
 * @param   none
 *
 * @return  none
 */
static void hidappDiscoverDevices( void )
{
  if ( hidappBLEState == BLE_STATE_SCANNING )
  {
    HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK ); // red led

    // Begin scanning
    VOID GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST );
  }
}

/*********************************************************************
 * @fn      hidappSetIdle
 *
 * @brief   Set the device to idle.
 *
 * @param   none
 *
 * @return  none
 */
static void hidappSetIdle( void )
{
  hidappBLEState = BLE_STATE_IDLE;

  HalLedSet( HAL_LED_2, HAL_LED_MODE_ON ); // red led
}

/*********************************************************************
 * @fn      hidappPairStateCB
 *
 * @brief   Notification from the Bond Manager on pairing state.
 *
 * @param   connHandle  - the connection handle this bonding event occured for.
 * @param   state       - new bonding state
 * @param   status      - success of failure of bonding
 *
 * @return  none
 */
static void hidappPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  switch( state )
  {
    case GAPBOND_PAIRING_STATE_BONDED:
      if ( status == SUCCESS )
      {
        // Enter a GAP Bond manager Paired state
        gapBondMgrState = PAIRED_BONDED_STATE;

        //Check if this is the same address as a previous connection
        if ( osal_memcmp( remoteHandles.lastRemoteAddr, remoteAddr, B_ADDR_LEN ) == TRUE  )
        {
          serviceChangeHandle = remoteHandles.svcChangeHandle;

          // Check if there has been a service change or if this is a newly connected device
          if ( ( remoteHandles.mouseCharHandle == GATT_INVALID_HANDLE )        ||
               ( remoteHandles.keyCharHandle == GATT_INVALID_HANDLE )          ||
               ( remoteHandles.consumerCtrlCharHandle == GATT_INVALID_HANDLE ) ||
               ( serviceChange == CHANGE_OCCURED ) )
          {
            // Do we know the service change handle yet?
            if (serviceChangeHandle == GATT_INVALID_HANDLE )
            {
              // Begin dicovery of GATT Service Changed characteristic
              serviceToDiscover = GATT_SERVICE_UUID;
            }
            else
            {
              // Begin discovery of HID service
              serviceToDiscover = HID_SERVICE_UUID;
            }

            // We must perform service discovery again, something might have changed.
            // Begin Service Discovery
            hidappDiscoverService( connHandle, serviceToDiscover );

            serviceDiscComplete = FALSE;
          }
          else
          {
            // No change, restore handle info.
            // bonding indicates that we probably already enabled all these characteristics. easy fix if not.
            serviceDiscComplete    = TRUE;
            mouseCharHandle        = remoteHandles.mouseCharHandle;
            keyCharHandle          = remoteHandles.keyCharHandle;
            consumerCtrlCharHandle = remoteHandles.consumerCtrlCharHandle;
            mouseCCCHandle         = remoteHandles.mouseCCCHandle;
            keyCCCHandle           = remoteHandles.keyCCCHandle;
            consumerCtrlCCCHandle  = remoteHandles.consumerCtrlCCCHandle;
          }
        }
        else if ( osal_isbufset( remoteHandles.lastRemoteAddr, 0x00, B_ADDR_LEN ) == TRUE )
        {
          // lastRemoteAddr is all 0's, which means the device was bonded before
          // it was power-cycled, and that we probably already enabled all CCCDs.
          // So, we only need to find out attribute report handles.
          enableCCCDs = FALSE;

          // Begin Service Discovery of HID Service to find out report handles
          serviceToDiscover = HID_SERVICE_UUID;
          hidappDiscoverService( connHandle, HID_SERVICE_UUID );
        }
      }
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if ( status == SUCCESS )
      {
        // Enter a GAP Bond manager Paired state
        gapBondMgrState = PAIRED_BONDED_STATE;

        // Begin Service Discovery of GATT Service
        serviceToDiscover = GATT_SERVICE_UUID;
        hidappDiscoverService( connHandle, GATT_SERVICE_UUID );
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      hidappEnableNotification
 *
 * @brief   Enable notification for a given attribute handle.
 *
 * @param   connHandle - connection handle to send notification on
 * @param   attrHandle - attribute handle to send notification for
 *
 * @return  none
 */
static void hidappEnableNotification( uint16 connHandle, uint16 attrHandle )
{
  attWriteReq_t req;
  uint8 notificationsOn[] = {0x01, 0x00};

  req.handle = attrHandle;

  req.len = 2;
  osal_memcpy(req.value, notificationsOn, 2);

  req.sig = 0;
  req.cmd = 0;

  VOID GATT_WriteCharValue( connHandle, &req, hidappTaskId );
}

/*********************************************************************
 * @fn      hidappDiscoverService
 *
 * @brief   Discover service using UUID.
 *
 * @param   connHandle - connection handle to do discovery on
 * @param   svcUuid - service UUID to discover
 *
 * @return  none
 */
static void hidappDiscoverService( uint16 connHandle, uint16 svcUuid )
{
  uint8 uuid[2] = {LO_UINT16(svcUuid), HI_UINT16(svcUuid)};

  VOID GATT_DiscPrimaryServiceByUUID( connHandle, uuid, ATT_BT_UUID_SIZE, hidappTaskId );
}

/*********************************************************************
 * @fn      hidappFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @param   uuid - service UUID to look for
 * @param   pData - received advertising data
 * @param   dataLen - advertising data length
 *
 * @return  TRUE if service UUID found
 */
static bool hidappFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
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
      if ( ( adType == GAP_ADTYPE_16BIT_MORE ) ||
           ( adType == GAP_ADTYPE_16BIT_COMPLETE ) )
      {
        pData++;
        adLen--;

        // For each UUID in list
        while ( ( adLen >= 2 ) && ( pData < pEnd ) )
        {
          // Check for match
          if ( ( pData[0] == LO_UINT16(uuid) ) &&
               ( pData[1] == HI_UINT16(uuid) ) )
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
 * @fn      hidappFindHIDRemote
 *
 * @brief   Search Scan Response data for a "HID AdvRemote"
 *
 * @param   pData - received advertising data
 * @param   dataLen - advertising data length
 *
 * @return  TRUE if found, false otherwise
 */
static uint8 hidappFindHIDRemote( uint8* pData, uint8 length )
{
  static uint8 remoteName[] =
  {
    'H',
    'I',
    'D',
    ' ',
    'A',
    'd',
    'v',
    'R',
    'e',
    'm',
    'o',
    't',
    'e'
  };

  // move pointer to the start of the scan response data.
  pData += 2;

  // adjust length as well
  length -= 2;

  return osal_memcmp( remoteName, pData, length );
}

/*********************************************************************
 * @fn      hidappSaveHandles
 *
 * @brief   save handle information in case next connection is to the
 *          same bonded device.
 *
 * @param   none.
 *
 * @return  none.
 */
static void hidappSaveHandles( void )
{
  // Service and Characteristic discovery variables.
  remoteHandles.mouseCharHandle        = mouseCharHandle;
  remoteHandles.keyCharHandle          = keyCharHandle;
  remoteHandles.consumerCtrlCharHandle = consumerCtrlCharHandle;

  // CCC's of the notifications
  remoteHandles.mouseCCCHandle         = mouseCCCHandle;
  remoteHandles.keyCCCHandle           = keyCCCHandle;
  remoteHandles.consumerCtrlCCCHandle  = consumerCtrlCCCHandle;
  remoteHandles.svcChangeHandle        = serviceChangeHandle;
}

/*********************************************************************
 * @fn      hidappEraseHandles
 *
 * @brief   erase handle information in case values as no longer important.
 *
 * @param   none.
 *
 * @return  none.
 */
static void hidappEraseHandles( void )
{
  // Service and Characteristic discovery variables
  remoteHandles.mouseCharHandle        = GATT_INVALID_HANDLE;
  remoteHandles.keyCharHandle          = GATT_INVALID_HANDLE;
  remoteHandles.consumerCtrlCharHandle = GATT_INVALID_HANDLE;

  // CCC's of the notifications
  remoteHandles.mouseCCCHandle         = GATT_INVALID_HANDLE;
  remoteHandles.keyCCCHandle           = GATT_INVALID_HANDLE;
  remoteHandles.consumerCtrlCCCHandle  = GATT_INVALID_HANDLE;
  remoteHandles.svcChangeHandle        = GATT_INVALID_HANDLE;

  // Erase the last connected device's address too
  osal_memset( remoteHandles.lastRemoteAddr, 0x00, B_ADDR_LEN );
}

/*********************************************************************
 * @fn      hidappBondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   none.
 *
 * @return  number of bonded devices.
 */
static uint8 hidappBondCount( void )
{
  uint8 bondCnt = 0;

  VOID GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCnt );

  return ( bondCnt );
}

/**************************************************************************************************
**************************************************************************************************/
