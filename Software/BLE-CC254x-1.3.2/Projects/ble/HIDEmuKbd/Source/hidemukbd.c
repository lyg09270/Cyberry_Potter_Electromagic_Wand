/**************************************************************************************************
  Filename:       hidemukbd.c

  Revised:        $Date: 2012-02-09 14:40:40 -0800 (Thu, 09 Feb 2012) $
  Revision:       $Revision: 65 $

  Description:    This file contains the HID emulated keyboard sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011-2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_led.h"
#include "hal_key.h"
#include "gatt.h"
#include "hci.h"
#include "npi.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "hidkbdservice.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidemukbd.h"
#include "hid_keys.h"

/*********************************************************************
 * MACROS
 */

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 hidEmuKbdTaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

const char ascii_to_hid_key_map[128][2] = {
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
    {0, 0}, {0, 0}, {0, KEY_ENTER}, {0, 0}, {0, KEY_TAB}, {0, 0}, {0, 0}, {0, 0},
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
    {0, KEY_SPACE}, {KEY_MOD_LSHIFT, KEY_1}, {KEY_MOD_LSHIFT, KEY_APOSTROPHE},
    {KEY_MOD_LSHIFT, KEY_3}, {KEY_MOD_LSHIFT, KEY_4}, {KEY_MOD_LSHIFT, KEY_5},
    {KEY_MOD_LSHIFT, KEY_7}, {0, KEY_APOSTROPHE}, {KEY_MOD_LSHIFT, KEY_9},
    {KEY_MOD_LSHIFT, KEY_0}, {KEY_MOD_LSHIFT, KEY_8}, {KEY_MOD_LSHIFT, KEY_EQUAL},
    {0, KEY_COMMA}, {0, KEY_MINUS}, {0, KEY_DOT}, {0, KEY_SLASH}, {0, KEY_0},
    {0, KEY_1}, {0, KEY_2}, {0, KEY_3}, {0, KEY_4}, {0, KEY_5}, {0, KEY_6},
    {0, KEY_7}, {0, KEY_8}, {0, KEY_9}, {KEY_MOD_LSHIFT, KEY_SEMICOLON},
    {0, KEY_SEMICOLON}, {KEY_MOD_LSHIFT, KEY_COMMA}, {0, KEY_EQUAL},
    {KEY_MOD_LSHIFT, KEY_DOT}, {KEY_MOD_LSHIFT, KEY_SLASH}, {KEY_MOD_LSHIFT, KEY_2},
    {KEY_MOD_LSHIFT, KEY_A}, {KEY_MOD_LSHIFT, KEY_B}, {KEY_MOD_LSHIFT, KEY_C},
    {KEY_MOD_LSHIFT, KEY_D}, {KEY_MOD_LSHIFT, KEY_E}, {KEY_MOD_LSHIFT, KEY_F},
    {KEY_MOD_LSHIFT, KEY_G}, {KEY_MOD_LSHIFT, KEY_H}, {KEY_MOD_LSHIFT, KEY_I},
    {KEY_MOD_LSHIFT, KEY_J}, {KEY_MOD_LSHIFT, KEY_K}, {KEY_MOD_LSHIFT, KEY_L},
    {KEY_MOD_LSHIFT, KEY_M}, {KEY_MOD_LSHIFT, KEY_N}, {KEY_MOD_LSHIFT, KEY_O},
    {KEY_MOD_LSHIFT, KEY_P}, {KEY_MOD_LSHIFT, KEY_Q}, {KEY_MOD_LSHIFT, KEY_R},
    {KEY_MOD_LSHIFT, KEY_S}, {KEY_MOD_LSHIFT, KEY_T}, {KEY_MOD_LSHIFT, KEY_U},
    {KEY_MOD_LSHIFT, KEY_V}, {KEY_MOD_LSHIFT, KEY_W}, {KEY_MOD_LSHIFT, KEY_X},
    {KEY_MOD_LSHIFT, KEY_Y}, {KEY_MOD_LSHIFT, KEY_Z}, {0, KEY_LEFTBRACE},
    {0, KEY_BACKSLASH}, {0, KEY_RIGHTBRACE}, {KEY_MOD_LSHIFT, KEY_6},
    {KEY_MOD_LSHIFT, KEY_MINUS}, {0, KEY_GRAVE}, {0, KEY_A}, {0, KEY_B},
    {0, KEY_C}, {0, KEY_D}, {0, KEY_E}, {0, KEY_F}, {0, KEY_G}, {0, KEY_H},
    {0, KEY_I}, {0, KEY_J}, {0, KEY_K}, {0, KEY_L}, {0, KEY_M}, {0, KEY_N},
    {0, KEY_O}, {0, KEY_P}, {0, KEY_Q}, {0, KEY_R}, {0, KEY_S}, {0, KEY_T},
    {0, KEY_U}, {0, KEY_V}, {0, KEY_W}, {0, KEY_X}, {0, KEY_Y}, {0, KEY_Z},
    {KEY_MOD_LSHIFT, KEY_LEFTBRACE}, {KEY_MOD_LSHIFT, KEY_BACKSLASH},
    {KEY_MOD_LSHIFT, KEY_RIGHTBRACE}, {KEY_MOD_LSHIFT, KEY_GRAVE}
};

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x0E,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'C',
  'y',
  'b',
  'e',
  'r',
  'r',
  'y',
  'p',
  'o',
  't',
  't',
  'e',
  'r'
};

// Advertising data
static uint8 advData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

  // service UUIDs
  0x07,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERVICE_UUID),
  HI_UINT16(HID_SERVICE_UUID),
  LO_UINT16(HID_BOOT_MOUSE_INPUT_UUID),
  HI_UINT16(HID_BOOT_MOUSE_INPUT_UUID),
  LO_UINT16(BATT_SERVICE_UUID),
  HI_UINT16(BATT_SERVICE_UUID)
};

// Device name attribute value
static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "CyberryPotter";

// HID Dev configuration
static hidDevCfg_t hidEmuKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

// TRUE if boot mouse enabled
static uint8 hidBootMouseEnabled = TRUE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void hidEmuKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidEmuKbd_HandleKeys( uint8 shift, uint8 keys );
static void hidEmuKbdSendReport( uint8 keycode ,uint8 modifier);
static void hidEmuKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y,uint8 whell);
static uint8 hidEmuKbdRcvReport( uint8 len, uint8 *pData );
static uint8 hidEmuKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData );
static void hidEmuKbdEvtCB( uint8 evt );

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuKbdHidCBs =
{
  hidEmuKbdRptCB,
  hidEmuKbdEvtCB,
  NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
 static void NpiSerialCallback( uint8 port, uint8 events );
/*********************************************************************
 * @fn      HidEmuKbd_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
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
void HidEmuKbd_Init( uint8 task_id )
{
  hidEmuKbdTaskId = task_id;
  NPI_InitTransport(NpiSerialCallback);
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );

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

  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8), &critical );
  }

  // Set up HID keyboard service
  HidKbd_AddService( );

  // Register for HID Dev callback
  HidDev_Register( &hidEmuKbdCfg, &hidEmuKbdHidCBs );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( hidEmuKbdTaskId );

#if defined( CC2540_MINIDK )
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

  // Setup a delayed profile startup
  osal_set_event( hidEmuKbdTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HidEmuKbd_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 HidEmuKbd_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidEmuKbdTaskId )) != NULL )
    {
      hidEmuKbd_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    return ( events ^ START_DEVICE_EVT );
  }

   return 0;
}

/*********************************************************************
 * @fn      hidEmuKbd_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidEmuKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  case KEY_CHANGE:
      hidEmuKbd_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

  default:
      break;
  }
}

/*********************************************************************
 * @fn      hidEmuKbd_HandleKeys
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
static void hidEmuKbd_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  (void)keys;
}

/*********************************************************************
 * @fn      hidEmuKbdSendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidEmuKbdSendReport( uint8 keycode ,uint8 modifier)
{
  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = modifier;  // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidEmuKbdSendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
static void hidEmuKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y,uint8 whell)
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

  buf[0] = buttons;   // Buttons
  buf[1] = x;         // X
  buf[2] = y;         // Y
  buf[3] = whell;         // Wheel
  buf[4] = 0;         // AC Pan

  HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                 HID_MOUSE_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidEmuKbdRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidEmuKbdRcvReport( uint8 len, uint8 *pData )
{
  // verify data length
  if ( len == HID_LED_OUT_RPT_LEN )
  {
    return SUCCESS;
  }
  else
  {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

/*********************************************************************
 * @fn      hidEmuKbdRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8 hidEmuKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData )
{
  uint8 status = SUCCESS;

  // write
  if ( oper == HID_DEV_OPER_WRITE )
  {
    if ( uuid == HID_REPORT_UUID )
    {
      // process write to LED output report; ignore others
      if ( type == HID_REPORT_TYPE_OUTPUT )
      {
        status = hidEmuKbdRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
  }
  // notifications enabled
  else if ( oper == HID_DEV_OPER_ENABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = TRUE;
    }
  }
  // notifications disabled
  else if ( oper == HID_DEV_OPER_DISABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = FALSE;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      hidEmuKbdEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidEmuKbdEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}

static void ASCII_2_Keyboard(uint8 asciiChar)
{
  // ASCII printable characters range from 32 to 126
    if (asciiChar <= 128) {
        // Calculate index in the map array
        uint8 keyIndex = asciiChar;
        uint8 keycode = ascii_to_hid_key_map[keyIndex][1];
        uint8 modifier = ascii_to_hid_key_map[keyIndex][0];
        uint8 i = 0;

        hidEmuKbdSendReport(keycode, modifier);  // Send the key without modifiers
        hidEmuKbdSendReport(KEY_NONE, KEY_NONE);
        
    }
}


static void NpiSerialCallback(uint8 port, uint8 events)
{
    (void)port;

    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))
    {
        uint8 numBytes = 0;
        uint8 i = 0;
        numBytes = NPI_RxBufLen();
        
        if (numBytes == 0)
        {
            return;
        }
        else
        {
            uint8 *buffer = osal_mem_alloc(numBytes);
            if (buffer)
            {
                NPI_ReadTransport(buffer, numBytes);   
                
                for(i = 0; i < numBytes;i++)
                {
                 ASCII_2_Keyboard(buffer[i]); 
                }

                osal_mem_free(buffer);
            }
        }
    }
}


/*********************************************************************
*********************************************************************/
