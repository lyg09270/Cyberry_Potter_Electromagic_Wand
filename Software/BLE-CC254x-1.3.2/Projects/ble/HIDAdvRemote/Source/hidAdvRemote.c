 /**************************************************************************************************
  Filename:       hidAdvRemote.c
  Revised:        $Date: 2012-11-12 16:08:16 -0800 (Mon, 12 Nov 2012) $
  Revision:       $Revision: 32163 $

  Description:    HID Advanced Remote Application.

  Copyright 2012 - 2013 Texas Instruments Incorporated. All rights reserved.

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
#include "osal_bufmgr.h"
#include "OnBoard.h"

#include "HAL_motion.h"
#include "hal_drivers.h"
#include "hal_buzzer.h"
#include "hal_key.h"
#include "hal_adc.h"

#include "hci.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "hidkbmservice.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"

#include "hidAdvRemote.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              60000

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         15

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE//FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_ONLY//GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

#define PASSCODE_LEN                          6

// HID Mouse input report length
#define HID_MOUSE_IN_RPT_LEN                  4

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN               8

// HID consumer control input report length
#define HID_CC_IN_RPT_LEN                     2

// HID LED output report length
#define HID_LED_OUT_RPT_LEN                   1

#define HID_CC_RPT_MAX_NUM_SELECTIONS         3

// Motion Sensor calibration time, in milliseconds
#define HAR_CAL_DURATION                      5000

// Selected HID LED bitmaps
#define LED_NUM_LOCK                          0x01
#define LED_CAPS_LOCK                         0x02

// Usage Page
#define USAGE_PAGE_UNDEFINED                  0x00 // RC control buttons
#define USAGE_PAGE_GD                         0x01 // Generic Desktop page
#define USAGE_PAGE_CC                         0x02 // Consumer Control page

#ifndef KEY_INT_ENABLED
  #define KEY_INT_ENABLED                     TRUE
#endif

#ifndef HAL_KEY_CODE_NOKEY
  #define HAL_KEY_CODE_NOKEY                  0xFF
#endif

// Target device type selection key
#define TARGET_TYPE                           0xC0

// Target Type List
#define TARGET_RESERVED_INVALID               0x00
#define TARGET_TV                             0x01
#define TARGET_FAV                            0x02
#define TARGET_VOD                            0x03

// Note that sequence has to be maintained to use function pointer array
#define ACT_START                             0xF0
#define ACT_PAIR                              0xF0
#define ACT_UNPAIR                            0xF1
#define ACT_AIR_MOUSE_CAL                     0xF2
#define ACT_MOUSE_RESOLUTION_DECREASE         0xF3
#define ACT_MOUSE_RESOLUTION_INCREASE         0xF4
#define ACT_AIR_MOUSE_CURSOR_CTL              0xF5

#define MOUSE_DATA_MAX                        127

// Task Events
#define START_DEVICE_EVT                      0x0001
#define DOUBLE_CLICK_TIMER_EVT                0x0002
#define MOTION_SENSOR_TIMER_EVT               0x0004
#define CAL_PROGRESS_TIMER_EVT                0x0008

#define DOUBLE_CLICK_OFF_TIMEOUT              500   // 500 ms
#define MOTION_SENSOR_OFF_TIMEOUT             5000  // 5 sec
#define CAL_PROGRESS_TIMEOUT                  1000  // 1 sec

// Time of no movement which causes transition from motion detection to low power, in ms
#define MOTION_SENSOR_EXIT_TIMEOUT            45000 // 45 sec

// ADC voltage levels
#define BATT_ADC_LEVEL_4_5V                   429 // 4.5 V
#define BATT_ADC_LEVEL_2_0V                   119 // 2.0 V

/*********************************************************************
 * TYPEDEFS
 */

// Motion detection state
enum
{
  MOTION_DETECTOR_DISABLED,
  MOTION_DETECTOR_STANDBY,
  MOTION_DETECTOR_ENABLED
};
typedef uint8 motionDetectorState_t;

// Function pointer map for special key actions
typedef void (*harActionFn_t)(void);

// Keycode mapping table
typedef struct
{
  uint8 keycode;
  uint8 usagePage;
} keycodeMap_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID for internal task/event processing
uint8 hidAdvRemote_TaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

 /*********************************************************************
 * LOCAL VARIABLES
 */

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanRspData[] =
{
  0x0E,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
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

static uint8 advertData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_GENERIC_HID),
  HI_UINT16(GAP_APPEARE_GENERIC_HID),

  // service UUIDs
  0x05,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERVICE_UUID),
  HI_UINT16(HID_SERVICE_UUID),
  LO_UINT16(BATT_SERVICE_UUID),
  HI_UINT16(BATT_SERVICE_UUID)
};

uint8 initial_advertising_enable = TRUE;
uint16 gapRole_AdvertOffTime = 20;

// GAP GATT Attributes
static uint8 attDeviceName[] = "HID AdvRemote\0";

// Device appearance
static uint16 appearance = GAP_APPEARE_GENERIC_HID;

// HID Dev configuration
static hidDevCfg_t hidAdvRemoteCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

static bool doubleClickTimerRunning;

// key state
static uint8 keyRepeated;
static bool disableMouseMovementAfterKeyRelease;

static uint16 arcNoMotionCount;
static bool arcNoMotionTimerRunning=FALSE;

// Keeps track of left/middle/right mouse buttons
static uint8 mouseButtonStates;
static uint8 savedMouseButtonStates = 0;
static bool blockMouseMovements;
static bool keyPressed;
static motionDetectorState_t arcMotionDetectorState = MOTION_DETECTOR_DISABLED;

static uint8 remainingPasscodeDigits = 0;
static uint32 passcode;

// HID Keycode map table
static CONST keycodeMap_t keycodeMap[48] =
{
  // 0b00 <KPb> <KPa>
  // row mapped    to P0 and P1
  // column mapped to shift register controlled by P0 and P2
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 00 0000
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 00 0001
  { HID_KEYBOARD_LEFT_GUI, USAGE_PAGE_GD },                // 0b00 00 0010   - MENU
  { HID_KEYBOARD_LEFT_ARROW, USAGE_PAGE_GD },              // 0b00 00 0011   - NAV LEFT
  { HID_CONSUMER_SCAN_PREV_TRK, USAGE_PAGE_CC },           // 0b00 00 0100   - REV
  { HID_MOUSE_BUTTON_LEFT, USAGE_PAGE_GD },                // 0b00 00 0101   - MOUSE LEFT
  { HID_CONSUMER_VOLUME_DOWN, USAGE_PAGE_CC },             // 0b00 00 0110   - VOL-
  { HID_CONSUMER_VOLUME_UP, USAGE_PAGE_CC },               // 0b00 00 0111   - VOL+
  { HID_CONSUMER_RECORD, USAGE_PAGE_CC },                  // 0b00 00 1000   - REC
  { HID_KEYBOARD_1, USAGE_PAGE_GD },                       // 0b00 00 1001   - 1
  { ACT_MOUSE_RESOLUTION_DECREASE, USAGE_PAGE_UNDEFINED }, // 0b00 00 1010   - AV
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 00 1011
  { ACT_UNPAIR, USAGE_PAGE_UNDEFINED },                    // 0b00 00 1100   - RED (K1)
  { HID_KEYBOARD_7, USAGE_PAGE_GD },                       // 0b00 00 1101   - 7
  { HID_KEYBOARD_4, USAGE_PAGE_GD },                       // 0b00 00 1110   - 4
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 00 1111
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_UNDEFINED },         // 0b00 01 0000   - FAV
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_UNDEFINED },         // 0b00 01 0001   - VOD
  { HID_KEYBOARD_DOWN_ARROW, USAGE_PAGE_GD },              // 0b00 01 0010   - NAV DOWN
  { HID_KEYBOARD_RETURN, USAGE_PAGE_GD },                  // 0b00 01 0011   - OK
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 01 0100
  { ACT_AIR_MOUSE_CURSOR_CTL, USAGE_PAGE_UNDEFINED },      // 0b00 01 0101   - MOUSE MIDDLE
  { HID_KEYBOARD_UP_ARROW, USAGE_PAGE_GD },                // 0b00 01 0110   - NAV UP
  { HID_CONSUMER_MUTE, USAGE_PAGE_CC },                    // 0b00 01 0111   - MUTE
  { HID_CONSUMER_PLAY, USAGE_PAGE_CC },                    // 0b00 01 1000   - PLAY
  { HID_KEYBOARD_2, USAGE_PAGE_GD },                       // 0b00 01 1001   - 2
  { HID_KEYBOARD_0, USAGE_PAGE_GD },                       // 0b00 01 1010   - 0
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_UNDEFINED },         // 0b00 01 1011   - YELLOW (K3)
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_UNDEFINED },         // 0b00 01 1100   - GREEN (K2)
  { HID_KEYBOARD_8, USAGE_PAGE_GD },                       // 0b00 01 1101   - 8
  { HID_KEYBOARD_5, USAGE_PAGE_GD },                       // 0b00 01 1110   - 5
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 01 1111
  { HID_CONSUMER_POWER, USAGE_PAGE_CC },                   // 0b00 10 0000   - POWER
  { TARGET_TYPE + TARGET_TV, USAGE_PAGE_UNDEFINED },       // 0b00 10 0001   - TV
  { HID_KEYBOARD_DELETE, USAGE_PAGE_GD },                  // 0b00 10 0010   - BACK
  { HID_KEYBOARD_RIGHT_ARROW, USAGE_PAGE_GD },             // 0b00 10 0011   - NAV RIGHT
  { HID_CONSUMER_SCAN_NEXT_TRK, USAGE_PAGE_CC },           // 0b00 10 0100   - FFWD
  { HID_MOUSE_BUTTON_RIGHT, USAGE_PAGE_GD },               // 0b00 10 0101   - MOUSE RIGHT
  { HID_CONSUMER_CHANNEL_DOWN, USAGE_PAGE_CC },            // 0b00 10 0110   - CH-
  { HID_CONSUMER_CHANNEL_UP, USAGE_PAGE_CC },              // 0b00 10 0111   - CH+
  { HID_CONSUMER_STOP, USAGE_PAGE_CC },                    // 0b00 10 1000   - STOP
  { HID_KEYBOARD_3, USAGE_PAGE_GD },                       // 0b00 10 1001   - 3
  { ACT_MOUSE_RESOLUTION_INCREASE, USAGE_PAGE_UNDEFINED }, // 0b00 10 1010   - PAUSE (?)
  { ACT_AIR_MOUSE_CAL, USAGE_PAGE_UNDEFINED },             // 0b00 10 1011   - BLUE (K4)
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD },                // 0b00 10 1100
  { HID_KEYBOARD_9, USAGE_PAGE_GD },                       // 0b00 10 1101   - 9
  { HID_KEYBOARD_6, USAGE_PAGE_GD },                       // 0b00 10 1110   - 6
  { HID_KEYBOARD_RESERVED, USAGE_PAGE_GD }                 // 0b00 10 1111
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void hidAdvRemote_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidAdvRemoteHandleKeys( uint8 keycode, bool keyPressed, uint8 keyRepeated );
static void hidAdvRemoteGetPasscode( uint8 keycode );
static void hidKeyboardSendReport( uint8 modifiers, uint8 keycode );
static void hidMouseSendReport( uint8 mouseStates, int8 mickeysX, int8 mickeysY );
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated );
static void hidCCBuildReport( uint8 *pBuf, uint8 cmd );

static void hidAdvRemotePairKeyAction( void );
static void hidAdvRemoteUnpairKeyAction( void );

static void hidAdvRemoteAirMouseCalKeyAction( void );
static void hidAdvRemoteAirMouseResolutionDecrease( void );
static void hidAdvRemoteAirMouseResolutionIncrease( void );
static void hidAdvRemoteAirMouseCursorCtlKeyPress( void );
static void hidAdvRemoteAirMouseCursorCtlKeyRelease( void );

static uint8 hidAdvRemoteRcvReport( uint8 len, uint8 *pData );
static uint8 hidAdvRemoteRptCB( uint8 id, uint8 type, uint16 uuid,
                                uint8 oper, uint8 *pLen, uint8 *pData );
static void hidAdvRemoteEvtCB( uint8 evt );
static void hidAdvRemotePasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                    uint8 uiInputs, uint8 uiOutputs );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// HID Dev callbacks
static hidDevCB_t hidAdvRemoteHidCBs =
{
  hidAdvRemoteRptCB,      // HID Report callback
  hidAdvRemoteEvtCB,      // HID event callback
  hidAdvRemotePasscodeCB  // HID Passcode callback
};

// Key action functions
static CONST harActionFn_t harSpecialKeyActions[] =
{
  hidAdvRemotePairKeyAction,
  hidAdvRemoteUnpairKeyAction,
  hidAdvRemoteAirMouseCalKeyAction,
  hidAdvRemoteAirMouseResolutionDecrease,
  hidAdvRemoteAirMouseResolutionIncrease,
  hidAdvRemoteAirMouseCursorCtlKeyPress
};

// HAL Related Callback functions
static void hidAdvRemoteKeyCback( uint8 keys, uint8 state );
static void hidAdvRemoteCalBuzzerRing( uint16 timeout, uint8 tone );
static void hidAdvRemoteCalCompleteCback( void );
static void hidAdvRemoteMotionSensorCback( int16 gyroMickeysX, int16 gyroMickeysY );
static void hidAdvRemoteBuzzerCompleteCback( void );

// Battery service callbacks
static void hidAdvRemoteBattMeasSetupCB( void );
static void hidAdvRemoteBattMeasTeardownCB( void );

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

 /*********************************************************************
 * @fn      HidAdvRemote_Init
 *
 * @brief   Initialization function for the Advance Remote App Task.
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
void HidAdvRemote_Init( uint8 task_id )
{
  hidAdvRemote_TaskId = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = FALSE;

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

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  {
    gapPeriConnectParams_t connParams =
    {
      DEFAULT_DESIRED_MIN_CONN_INTERVAL,
      DEFAULT_DESIRED_MAX_CONN_INTERVAL,
      DEFAULT_DESIRED_SLAVE_LATENCY,
      DEFAULT_DESIRED_CONN_TIMEOUT
    };

    GGS_SetParameter( GGS_PERI_CONN_PARAM_ATT, sizeof( gapPeriConnectParams_t ), (void *)&connParams );
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof ( attDeviceName ) , attDeviceName );
    GGS_SetParameter( GGS_APPEARANCE_ATT, sizeof( uint16 ), (void *)&appearance );
  }

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

    Batt_Setup( HAL_ADC_CHN_AIN6, BATT_ADC_LEVEL_2_0V, BATT_ADC_LEVEL_4_5V,
                hidAdvRemoteBattMeasSetupCB, hidAdvRemoteBattMeasTeardownCB, NULL );

    /* Control of transistor in the battery measurement voltage divider */
    P1DIR |= (1<<5);
    P1SEL &= ~(1<<5);
    P1_5 = 0; // Stop feeding current through voltage divider
    
    /* Analog measuring pin setup. This will be retained during ADC read */
    P0DIR |= (1<<6);
    P0SEL &= ~(1<<6);
    P0_6 = 0; // LOW output to stop leakage current due to default pull-up
  }

  // Set up HID keyboard/mouse service
  HidKbM_AddService();

  // Register for HID Dev callback
  HidDev_Register( &hidAdvRemoteCfg, &hidAdvRemoteHidCBs );

  HalKeyInit();
  HalKeyConfig( KEY_INT_ENABLED, hidAdvRemoteKeyCback );
  HalMotionConfig( hidAdvRemoteMotionSensorCback );

  VOID HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE );

  // Setup a delayed profile startup
  osal_start_timerEx( hidAdvRemote_TaskId, START_DEVICE_EVT, 500 );
}

/*********************************************************************
 * @fn      HidAdvRemote_ProcessEvent
 *
 * @brief   HID Advance Remote Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 HidAdvRemote_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidAdvRemote_TaskId )) != NULL )
    {
      hidAdvRemote_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    HalKeyInit();
    HalKeyConfig( KEY_INT_ENABLED, hidAdvRemoteKeyCback );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & DOUBLE_CLICK_TIMER_EVT )
  {
    /* No special processing needed; just indicate timer no longer running */
    doubleClickTimerRunning = FALSE;

    return ( events ^ DOUBLE_CLICK_TIMER_EVT );
  }

  if ( events & MOTION_SENSOR_TIMER_EVT )
  {
    /* user hasn't performed mouse activity -- turn off motion sensor system */
    HalMotionDisable();
    arcMotionDetectorState = MOTION_DETECTOR_DISABLED;

    /* enable  power management */
    osal_pwrmgr_device( PWRMGR_BATTERY );

    return ( events ^ MOTION_SENSOR_TIMER_EVT );
  }

  if ( events & CAL_PROGRESS_TIMER_EVT )
  {
    hidAdvRemoteCalBuzzerRing( 1, HAL_BUZZER_HIGH_TONE ); // short high tone

    return ( events ^ CAL_PROGRESS_TIMER_EVT );
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      hidAdvRemote_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidAdvRemote_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  return;
}

/*********************************************************************
 *
 * @fn      hidAdvRemoteKeyCback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - key that was pressed (i.e. the scanned row/col index)
 *          state - shifted
 *
 * @return  void
 */
static void hidAdvRemoteKeyCback(uint8 keys, uint8 state)
{
  static uint8 lastKey = HAL_KEY_CODE_NOKEY;
  static uint8 lastCmd = HID_KEYBOARD_RESERVED;
  bool sendMouseReport = FALSE;

  // Get the key code based on the row/col index
  uint8 cmd = keycodeMap[keys].keycode;

  (void) state; // unused argument

  if ( keys == HAL_KEY_CODE_NOKEY )
  {
    // Overwrite the invalid cmd lookup on key release.
    cmd = HID_KEYBOARD_RESERVED;
    keyRepeated = 0;

    if ( mouseButtonStates != 0 )
    {
      /* While mouse buttons are sensed as buttons, they are reported in
       * HID Mouse Reports, which are sent from the HAR_MotionSensorCback
       * function. The global "mouseButtonStates" keeps track of the state
       * of the mouse buttons, and this callback function simply sets/clears
       * the state of these buttons.
       */
      lastKey = HAL_KEY_CODE_NOKEY;
      mouseButtonStates = 0;
      sendMouseReport = TRUE;
    }
    else if ( lastCmd == ACT_AIR_MOUSE_CURSOR_CTL )
    {
      hidAdvRemoteAirMouseCursorCtlKeyRelease();
      lastCmd = HID_KEYBOARD_RESERVED;
    }
  }
  else if ( lastKey == keys )
  {
    // same key is still pressed
    if ( keyRepeated < 255 )
    {
      keyRepeated++;
    }
  }
  else
  {
    // remember last row/col index
    lastKey = keys;
    lastCmd = cmd;
    keyRepeated = 0;
  }

  /* Mouse button clicks are simply recorded in the global "mouseButtonStates"
   * and reported by the motion sensor callback function.
   */
  if ( cmd == HID_MOUSE_BUTTON_LEFT )
  {
    mouseButtonStates |= 0x01;
    sendMouseReport = TRUE;
  }
  else if ( cmd == HID_MOUSE_BUTTON_RIGHT )
  {
    mouseButtonStates |= 0x02;
    sendMouseReport = TRUE;
  }
  else if ( cmd >= ACT_START )
  {
    // action table command
    harSpecialKeyActions[cmd - ACT_START]();
  }
  else if ( sendMouseReport == FALSE ) // we have a valid key command
  {
    uint8 key;
    uint8 usagePage;

    if ( keys == HAL_KEY_CODE_NOKEY )
    {
      // indicate no key pressed anymore
      key = 0;
      keyPressed = FALSE;

      // Get the last key code based on the row/col index.
      cmd = keycodeMap[lastKey].keycode;
      usagePage = keycodeMap[lastKey].usagePage;
      lastKey = HAL_KEY_CODE_NOKEY;
    }
    else  // key pressed
    {
      // insert key that was pressed
      key = cmd;
      keyPressed = TRUE;

      usagePage = keycodeMap[keys].usagePage;
    }

    // Only send out functional keys
    if ( usagePage == USAGE_PAGE_GD )
    {
      // Keyboard key
      hidAdvRemoteHandleKeys( key, keyPressed, keyRepeated );
    }
    else if ( usagePage == USAGE_PAGE_CC )
    {
      // Consumer control key
      hidCCSendReport( cmd, keyPressed, keyRepeated );
    }
    else
    {
      // Control button
    }
  }

  if ( ( sendMouseReport == TRUE )                         &&
       ( arcMotionDetectorState == MOTION_DETECTOR_DISABLED ) &&
       ( mouseButtonStates != savedMouseButtonStates ) )
  {
    hidMouseSendReport( mouseButtonStates, 0, 0 );

    savedMouseButtonStates = mouseButtonStates;
  }
}

/*********************************************************************
 * @fn      hidAdvRemoteHandleKeys
 *
 * @brief   Handle keys pressed
 *
 * @param   keycode - HID keycode
 *
 * @return  none
 */
static void hidAdvRemoteHandleKeys( uint8 keycode, bool keyPressed, uint8 keyRepeated )
{
  // Is the user in the middle of entering a passcode?
  if ( remainingPasscodeDigits == 0 )
  {
    if ( keycode == HID_KEYBOARD_LEFT_GUI )
    {
      // Only left GUI modifier is supported
      hidKeyboardSendReport( (0x01 << 3), HID_KEYBOARD_RESERVED );
    }
    else
    {
      // no keyboard modifiers
      hidKeyboardSendReport( HID_KEYBOARD_RESERVED, keycode );
    }
  }
  else
  {
    if ( keyRepeated == 0 )
    {
      hidAdvRemoteGetPasscode( keycode );
    }
  }
}

/*********************************************************************
 * @fn      hidAdvRemoteGetPasscode
 *
 * @brief   Build and send a passcode.
 *
 * @param   keycode - HID keycode
 *
 * @return  none
 */
static void hidAdvRemoteGetPasscode( uint8 keycode )
{
  if ( keycode != HID_KEYBOARD_RESERVED )
  {
    if ( ( keycode >= HID_KEYBOARD_1 ) && ( keycode <= HID_KEYBOARD_0 ) )
    {
      // Append new digit to passcode
      passcode *= 10;
      passcode += ( ( keycode - HID_KEYBOARD_1 + 1 ) % 10 );

      if ( --remainingPasscodeDigits == 0 )
      {
         // Send passcode response
        HidDev_PasscodeRsp( SUCCESS, passcode );
      }
    }
    else
    {
      // Send passcode response
      HidDev_PasscodeRsp( SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED, passcode );

      remainingPasscodeDigits = 0;
    }
  }
}

/*********************************************************************
 * @fn      hidKeyboardSendReport
 *
 * @brief   Build and send a HID Keyboard report.
 *
 * @param   modifiers - HID modifiers.
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidKeyboardSendReport( uint8 modifiers, uint8 keycode )
{
  static uint8 prevKeycode = HAL_KEY_CODE_NOKEY;
  static uint8 prevModifiers = HID_KEYBOARD_RESERVED;

  // Only send the report if something meaningful to report
  if ( ( prevKeycode != keycode ) || ( prevModifiers != modifiers ) )
  {
    uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

    // No need to include Report Id
    buf[0] = modifiers;         // Modifier keys
    buf[1] = 0;                 // Reserved
    buf[2] = keycode;           // Keycode 1
    buf[3] = 0;                 // Keycode 2
    buf[4] = 0;                 // Keycode 3
    buf[5] = 0;                 // Keycode 4
    buf[6] = 0;                 // Keycode 5
    buf[7] = 0;                 // Keycode 6

    HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                   HID_KEYBOARD_IN_RPT_LEN, buf );

    // Save the new values
    prevKeycode = keycode;
    prevModifiers = modifiers;
  }
}

/*********************************************************************
 * @fn      hidMouseSendReport
 *
 * @brief   Build and send a HID Mouse report.
 *
 * @param   mouseStates - bitmap of left/middle/right mouse button states
 * @param   mickeysX - amount of mouse movement along X-axis in units of Mickeys (1/200 of an inch)
 * @param   mickeysY - amount of mouse movement along Y-axis
 *
 * @return  none
 */
static void hidMouseSendReport( uint8 mouseStates, int8 mickeysX, int8 mickeysY )
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

  // No need to include Report Id
  buf[0] = mouseStates;         // Buttons
  buf[1] = mickeysX;            // X
  buf[2] = mickeysY;            // Y
  buf[3] = 0;                   // Wheel

  HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                 HID_MOUSE_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidCCSendReport
 *
 * @brief   Build and send a HID Consumer Control report.
 *
 * @param   cmd - bitmap of left/middle/right mouse button states
 * @param   keyPressed - amount of mouse movement along X-axis in units of Mickeys (1/200 of an inch)
 * @param   keyRepeated - amount of mouse movement along Y-axis
 *
 * @return  none
 */
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated )
{
  // Only send the report if something meaningful to report
  if ( keyRepeated == 0 )
  {
    uint8 buf[HID_CC_IN_RPT_LEN] = { 0, 0 };

    // No need to include Report Id
    if ( keyPressed )
    {
      hidCCBuildReport( buf, cmd );
    }

    HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                   HID_CC_IN_RPT_LEN, buf );
  }
}

/*********************************************************************
 * @fn      hidCCBuildReport
 *
 * @brief   Build a HID Consumer Control report.
 *
 * @param   pBuf - report to be built
 * @param   cmd - pressed command
 *
 * @return  none
 */
static void hidCCBuildReport( uint8 *pBuf, uint8 cmd )
{
  switch ( cmd )
  {
    case HID_CONSUMER_CHANNEL_UP:
      HID_CC_RPT_SET_CHANNEL( pBuf, HID_CC_RPT_CHANNEL_UP );
      break;

    case HID_CONSUMER_CHANNEL_DOWN:
      HID_CC_RPT_SET_CHANNEL( pBuf, HID_CC_RPT_CHANNEL_DOWN );
      break;

    case HID_CONSUMER_VOLUME_UP:
      HID_CC_RPT_SET_VOLUME_UP( pBuf );
      break;

    case HID_CONSUMER_VOLUME_DOWN:
      HID_CC_RPT_SET_VOLUME_DOWN( pBuf );
      break;

    case HID_CONSUMER_MUTE:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_MUTE );
      break;

    case HID_CONSUMER_POWER:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_POWER );
      break;

    case HID_CONSUMER_RECALL_LAST:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_LAST );
      break;

    case HID_CONSUMER_ASSIGN_SEL:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_ASSIGN_SEL );
      break;

    case HID_CONSUMER_PLAY:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_PLAY );
      break;

    case HID_CONSUMER_PAUSE:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_PAUSE );
      break;

    case HID_CONSUMER_RECORD:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_RECORD );
      break;

    case HID_CONSUMER_FAST_FORWARD:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_FAST_FWD );
      break;

    case HID_CONSUMER_REWIND:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_REWIND );
      break;

    case HID_CONSUMER_SCAN_NEXT_TRK:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_SCAN_NEXT_TRK );
      break;

    case HID_CONSUMER_SCAN_PREV_TRK:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_SCAN_PREV_TRK );
      break;

    case HID_CONSUMER_STOP:
      HID_CC_RPT_SET_BUTTON( pBuf, HID_CC_RPT_STOP );
      break;

    default:
      if ( ( cmd >= HID_KEYBOARD_1 ) && ( cmd <= HID_KEYBOARD_0 ) )
      {
        HID_CC_RPT_SET_BUTTON( pBuf, (cmd - HID_KEYBOARD_1 + 1) % 10 );
      }
      break;
  }
}

/*********************************************************************
 * @fn          hidAdvRemoteAirMouseCursorCtlKeyPress
 *
 * @brief       This function performs action required when the Air
 *              Mouse control key is pressed.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteAirMouseCursorCtlKeyPress( void )
{
  if ( keyRepeated == 0 ) // only act on initial keypress detection
  {
    // cancel motion sensor poweroff timer
    osal_stop_timerEx( hidAdvRemote_TaskId, MOTION_SENSOR_TIMER_EVT );

    arcNoMotionCount = 0;
    blockMouseMovements = FALSE;
    disableMouseMovementAfterKeyRelease = TRUE;
    arcNoMotionTimerRunning = FALSE;

    if ( arcMotionDetectorState != MOTION_DETECTOR_ENABLED )
    {
      HalMotionEnable();
      arcMotionDetectorState = MOTION_DETECTOR_ENABLED;

      /* disable power management */
      osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    }
  }
}

/*********************************************************************
 * @fn          hidAdvRemoteAirMouseCursorCtlKeyRelease
 *
 * @brief       This function performs action required when the Air
 *              Mouse cursor control key is released.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteAirMouseCursorCtlKeyRelease( void )
{
  /* Releasing air mouse cursor control key. Perform double click processing. */
  if ( doubleClickTimerRunning == TRUE )
  {
    /* key released again within double click time, so double click has occurred */
    disableMouseMovementAfterKeyRelease = FALSE;
  }
  else
  {
    /* Start double click and motion sensor power off timers */
    doubleClickTimerRunning = TRUE;

    osal_start_timerEx( hidAdvRemote_TaskId,
                        DOUBLE_CLICK_TIMER_EVT,
                        DOUBLE_CLICK_OFF_TIMEOUT );

    osal_start_timerEx( hidAdvRemote_TaskId,
                        MOTION_SENSOR_TIMER_EVT,
                        MOTION_SENSOR_OFF_TIMEOUT );
  }

  /* Now determine what we should do with mouse movements */
  if ( disableMouseMovementAfterKeyRelease == TRUE )
  {
    blockMouseMovements = TRUE;
  }
}

/*********************************************************************
 * @fn          hidAdvRemotePairKeyAction
 *
 * @brief       This function performs action required when a pair
 *              trigger key is depressed.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemotePairKeyAction(void)
{

}

/*********************************************************************
 * @fn          harAirMouseCalKeyAction
 *
 * @brief       This function performs action required when the Air
 *              Mouse key is depressed.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteAirMouseCalKeyAction( void )
{
  if ( keyRepeated == 0 ) // only act on initial keypress detection
  {
    if ( HalMotionCal( hidAdvRemoteCalCompleteCback, HAR_CAL_DURATION ) == TRUE )
    {
      hidAdvRemoteCalBuzzerRing( 1, HAL_BUZZER_HIGH_TONE ); // short high tone

      osal_start_reload_timer( hidAdvRemote_TaskId,
                               CAL_PROGRESS_TIMER_EVT,
                               CAL_PROGRESS_TIMEOUT );
    }
    else
    {
      // Motion sensor calibration not started
      hidAdvRemoteCalBuzzerRing( 500, HAL_BUZZER_LOW_TONE ); // long low tone
    }
  }
}

/*********************************************************************
 * @fn          hidAdvRemoteAirMouseResolutionDecrease
 *
 * @brief       This function decreases the resolution (i.e. speed) of
 *              Air Mouse movement.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteAirMouseResolutionDecrease( void )
{
  /* Simply call HAL function to modify resolution */
  if ( keyRepeated == 0 )
  {
    HalMotionModifyAirMouseResolution( HAL_MOTION_RESOLUTION_DECREASE );
  }
}

/*********************************************************************
 * @fn          hidAdvRemoteAirMouseResolutionIncrease
 *
 * @brief       This function increases the resolution (i.e. speed) of
 *              Air Mouse movement.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteAirMouseResolutionIncrease( void )
{
  /* Simply call HAL function to modify resolution */
  if ( keyRepeated == 0 )
  {
    HalMotionModifyAirMouseResolution( HAL_MOTION_RESOLUTION_INCREASE );
  }
}

/*********************************************************************
 *
 * @fn      hidAdvRemoteCalBuzzerRing
 *
 * @brief   Function to inform app of failure or progress of motion
 *          sensor calibration.
 *
 * @param   timeout - number of msec to ring the buzzer
 * @param   tone - type of tone (low or high)
 *
 * @return  void
 */
static void hidAdvRemoteCalBuzzerRing( uint16 timeout, uint8 tone )
{
  /* Provide feedback that calibration is complete */
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  /* Tell OSAL to not go to sleep because buzzer uses T3 */
  osal_pwrmgr_device( PWRMGR_ALWAYS_ON );

  /* Ring buzzer */
  HalBuzzerRing( timeout, tone, hidAdvRemoteBuzzerCompleteCback );
#endif
}

/*********************************************************************
 *
 * @fn      hidAdvRemoteCalCompleteCback
 *
 * @brief   Callback service to inform app of completion of motion
 *          sensor calibration.
 *
 * @param   none
 *
 * @return  void
 */
static void hidAdvRemoteCalCompleteCback( void )
{
  osal_stop_timerEx( hidAdvRemote_TaskId, CAL_PROGRESS_TIMER_EVT );

  hidAdvRemoteCalBuzzerRing( 500, HAL_BUZZER_HIGH_TONE ); // long high tone
}

/*********************************************************************
 *
 * @fn      hidAdvRemoteBuzzerCompleteCback
 *
 * @brief   Callback function called when ringing of buzzer is done.
 *
 * @param   none
 *
 * @return  void
 */
static void hidAdvRemoteBuzzerCompleteCback( void )
{
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  /* Tell OSAL it's OK to go to sleep */
  osal_pwrmgr_device( /*PWRMGR_ALWAYS_ON*/ PWRMGR_BATTERY );
#endif
}

/*********************************************************************
 * @fn          hidAdvRemoteUnpairKeyAction
 *
 * @brief       This function performs action required when un-pair
 *              trigger key is depressed.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
static void hidAdvRemoteUnpairKeyAction(void)
{
  HidDev_SetParameter( HIDDEV_ERASE_ALLBONDS, 0, NULL );
}

/*********************************************************************
 *
 * @fn      hidAdvRemoteMotionSensorCback
 *
 * @brief   Callback service for motion sensor samples
 *
 * @param   gyroMickeysX - X-axis movement in units of "Mickeys" (approx 1/200 inch per Mickey)
 *          gyroMickeysY - Y-axis movement
 *
 * @return  void
 */
static void hidAdvRemoteMotionSensorCback( int16 gyroMickeysX, int16 gyroMickeysY )
{

  /* Check for no motion for 10 seconds. If true, then place motion detection
   * hardware in standby mode.
   */
  if ( ( gyroMickeysX == 0 ) && ( gyroMickeysY == 0 ) )
  {
    arcNoMotionCount += 1;
    if ( arcNoMotionCount >= 1000 ) // samples arrive every 10 msec
    {
      HalMotionStandby();
      arcMotionDetectorState = MOTION_DETECTOR_STANDBY;
      arcNoMotionCount = 0;
      osal_start_timerEx(hidAdvRemote_TaskId, MOTION_SENSOR_TIMER_EVT, MOTION_SENSOR_EXIT_TIMEOUT);
      arcNoMotionTimerRunning = TRUE;
    }
  }
  else
  {
    arcMotionDetectorState = MOTION_DETECTOR_ENABLED;
    arcNoMotionCount = 0;
    if (arcNoMotionTimerRunning == TRUE)
    {
      /* cancel motion sensor poweroff timer */
      osal_stop_timerEx(hidAdvRemote_TaskId, MOTION_SENSOR_TIMER_EVT);
      arcNoMotionTimerRunning = FALSE;
    }
  }

  /* Make sure movement data is within bounds of report fields */
  if ( gyroMickeysX > MOUSE_DATA_MAX )
  {
    gyroMickeysX = MOUSE_DATA_MAX;
  }
  else if ( gyroMickeysX < -MOUSE_DATA_MAX )
  {
    gyroMickeysX = -MOUSE_DATA_MAX;
  }

  if ( gyroMickeysY > MOUSE_DATA_MAX )
  {
    gyroMickeysY = MOUSE_DATA_MAX;
  }
  else if ( gyroMickeysY < -MOUSE_DATA_MAX )
  {
    gyroMickeysY = -MOUSE_DATA_MAX;
  }

  /* Only send the report if something meaningful to report */
  if ( ( mouseButtonStates != savedMouseButtonStates ) || // mouse button change
       ( ( blockMouseMovements == FALSE )              && // mouse movements are allowed
         ( keyPressed == FALSE )                       && // freeze mouse movements if key pressed
         ( ( gyroMickeysX != 0 )       || // mouse movement was detected
           ( gyroMickeysY != 0 ) ) ) )
  {
    if ( blockMouseMovements == TRUE )
    {
      // Make sure X and Y movements are set to 0
      gyroMickeysX = 0;
      gyroMickeysY = 0;
    }

    hidMouseSendReport( mouseButtonStates,
                        (int8)gyroMickeysX,
                        (int8)gyroMickeysY );

    savedMouseButtonStates = mouseButtonStates;
  }
}

/*********************************************************************
 * @fn      hidAdvRemoteRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidAdvRemoteRcvReport( uint8 len, uint8 *pData )
{
  // No lEDS on the remote control
  return SUCCESS;
}

/*********************************************************************
 * @fn      hidAdvRemoteRptCB
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
static uint8 hidAdvRemoteRptCB( uint8 id, uint8 type, uint16 uuid,
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
        status = hidAdvRemoteRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbM_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbM_GetParameter( id, type, uuid, pLen, pData );
  }

  return status;
}

/*********************************************************************
 * @fn      hidAdvRemoteEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidAdvRemoteEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}

/*********************************************************************
 * @fn      hidAdvRemotePasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either public or random.
 * @param   connectionHandle - connection handle
 * @param   uiInputs - pairing User Interface Inputs - Ask user to input passcode
 * @param   uiOutputs - pairing User Interface Outputs - Display passcode
 *
 * @return  none
 */
static void hidAdvRemotePasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                   uint8 uiInputs, uint8 uiOutputs )
{
  remainingPasscodeDigits = PASSCODE_LEN;
  passcode = 0;
}

/*********************************************************************
 * @fn      hidAdvRemoteBattMeasSetupCB
 *
 * @brief   Callback from battery service to set up measurement environment
 *
 * @param   none
 *
 * @return  none
 */
static void hidAdvRemoteBattMeasSetupCB( void )
{
  P1_5 = 1;  // Open transistor in voltage divider
  
  /* Analog pin P0.6 is configured by HAL ADC routine */
}

/*********************************************************************
 * @fn      hidAdvRemoteBattMeasTeardownCB
 *
 * @brief   Callback from battery service to tear down measurement environment
 *
 * @param   none
 *
 * @return  none
 */
static void hidAdvRemoteBattMeasTeardownCB( void )
{
  P1_5 = 0; // Stop feeding current through voltage divider
  
  /* Idle state P0.6 settings are retained from Init */
}

/*********************************************************************
*********************************************************************/