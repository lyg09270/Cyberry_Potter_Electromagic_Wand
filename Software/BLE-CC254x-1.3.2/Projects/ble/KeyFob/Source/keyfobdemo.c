/**************************************************************************************************
  Filename:       keyfobdemo.c
  Revised:        $Date: 2013-05-01 13:58:23 -0700 (Wed, 01 May 2013) $
  Revision:       $Revision: 34101 $

  Description:    Key Fob Demo Application.

  Copyright 2009 - 2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "buzzer.h"

#if defined ( ACC_BMA250 )
#  include "bma250.h"
#elif defined ( ACC_CMA3000 )
#  include "cma3000d.h"
#endif

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "devinfoservice.h"
#include "proxreporter.h"
#include "battservice.h"
#include "accelerometer.h"
#include "simplekeys.h"

#include "keyfobdemo.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Delay between power-up and starting advertising (in ms)
#define STARTDELAY                            500

// Number of beeps before buzzer stops by itself
#define BUZZER_MAX_BEEPS                      10

// Buzzer beep tone frequency for "High Alert" (in Hz)
#define BUZZER_ALERT_HIGH_FREQ                4096

// Buzzer beep tone frequency for "Low Alert" (in Hz)
#define BUZZER_ALERT_LOW_FREQ                 250

// How often to check battery voltage (in ms)
#define BATTERY_CHECK_PERIOD                  10000

// How often (in ms) to read the accelerometer
#define ACCEL_READ_PERIOD                     50

// Minimum change in accelerometer before sending a notification
#define ACCEL_CHANGE_THRESHOLD                5

//GAP Peripheral Role desired connection parameters

// Use limited discoverable mode to advertise for 30.72s, and then stop, or
// use general discoverable mode to advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         5

// keyfobProximityState values
#define KEYFOB_PROXSTATE_INITIALIZED          0   // Advertising after initialization or due to terminated link
#define KEYFOB_PROXSTATE_CONNECTED_IN_RANGE   1   // Connected and "within range" of the master, as defined by
                                                  // proximity profile
#define KEYFOB_PROXSTATE_PATH_LOSS            2   // Connected and "out of range" of the master, as defined by
                                                  // proximity profile
#define KEYFOB_PROXSTATE_LINK_LOSS            3   // Disconnected as a result of a supervision timeout

// buzzer_state values
#define BUZZER_OFF                            0
#define BUZZER_ON                             1

// keyfobAlertState values
#define ALERT_STATE_OFF                       0
#define ALERT_STATE_LOW                       1
#define ALERT_STATE_HIGH                      2

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

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
static uint8 keyfobapp_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Proximity State Variables
static uint8 keyfobProxLLAlertLevel = PP_ALERT_LEVEL_NO;     // Link Loss Alert
static uint8 keyfobProxIMAlertLevel = PP_ALERT_LEVEL_NO;     // Link Loss Alert
static int8  keyfobProxTxPwrLevel = 0;  // Tx Power Level (0dBm default)

// keyfobProximityState is the current state of the device
static uint8 keyfobProximityState;

static uint8 keyfobAlertState;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 deviceName[] =
{
  // complete name
  0x0b,   // length of first data structure (11 bytes excluding length byte)
  0x09,   // AD Type = Complete local name
  0x4b,   // 'K'
  0x65,   // 'e'
  0x79,   // 'y'
  0x66,   // 'f'
  0x6f,   // 'o'
  0x62,   // 'b'
  0x64,   // 'd'
  0x65,   // 'e'
  0x6d,   // 'm'
  0x6f,   // 'o'
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  0x02,   // length of first data structure (2 bytes excluding length byte)
  GAP_ADTYPE_FLAGS,   // AD Type = Flags
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x07,   // length of second data structure (7 bytes excluding length byte)
  GAP_ADTYPE_16BIT_MORE,   // list of 16-bit UUID's available, but not complete list
  LO_UINT16( LINK_LOSS_SERVICE_UUID ),        // Link Loss Service (Proximity Profile)
  HI_UINT16( LINK_LOSS_SERVICE_UUID ),
  LO_UINT16( IMMEDIATE_ALERT_SERVICE_UUID ),  // Immediate Alert Service (Proximity / Find Me Profile)
  HI_UINT16( IMMEDIATE_ALERT_SERVICE_UUID ),
  LO_UINT16( TX_PWR_LEVEL_SERVICE_UUID ),     // Tx Power Level Service (Proximity Profile)
  HI_UINT16( TX_PWR_LEVEL_SERVICE_UUID )
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "TI BLE Keyfob";
  
// Buzzer state
static uint8 buzzer_state = BUZZER_OFF;
static uint8 buzzer_beep_count = 0;

// Accelerometer Profile Parameters
static uint8 accelEnabler = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void keyfobapp_PerformAlert( void );
static void keyfobapp_StopAlert( void );
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void proximityAttrCB( uint8 attrParamID );
static void accelEnablerChangeCB( void );
static void accelRead( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t keyFob_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                // When a valid RSSI is read from controller
};


// GAP Bond Manager Callbacks
static gapBondCBs_t keyFob_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Proximity Peripheral Profile Callbacks
static proxReporterCBs_t keyFob_ProximityCBs =
{
  proximityAttrCB,              // Whenever the Link Loss Alert attribute changes
};

// Accelerometer Profile Callbacks
static accelCBs_t keyFob_AccelCBs =
{
  accelEnablerChangeCB,          // Called when Enabler attribute changes
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      KeyFobApp_Init
 *
 * @brief   Initialization function for the Key Fob App Task.
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
void KeyFobApp_Init( uint8 task_id )
{
  keyfobapp_TaskID = task_id;
    
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
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
  
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( deviceName ), deviceName );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Attributes
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService();   // Device Information Service
  ProxReporter_AddService( GATT_ALL_SERVICES );  // Proximity Reporter Profile
  Batt_AddService( );     // Battery Service
  Accel_AddService( GATT_ALL_SERVICES );      // Accelerometer Profile
  SK_AddService( GATT_ALL_SERVICES );         // Simple Keys Profile

  keyfobProximityState = KEYFOB_PROXSTATE_INITIALIZED;

  // Initialize Tx Power Level characteristic in Proximity Reporter
  {
    int8 initialTxPowerLevel = 0;
    
    ProxReporter_SetParameter( PP_TX_POWER_LEVEL, sizeof ( int8 ), &initialTxPowerLevel );
  }

  keyfobAlertState = ALERT_STATE_OFF;

  // make sure buzzer is off
  buzzerStop();

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0x40; // Configure Port 1 as GPIO, except P1.6 for peripheral function for buzzer
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low


  // initialize the ADC for battery reads
  HalAdcInit();

  // Register for all key events - This app will handle all key events
  RegisterForKeys( keyfobapp_TaskID );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )
  
  // Setup a delayed profile startup
  osal_start_timerEx( keyfobapp_TaskID, KFD_START_DEVICE_EVT, STARTDELAY );
}

/*********************************************************************
 * @fn      KeyFobApp_ProcessEvent
 *
 * @brief   Key Fob Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 KeyFobApp_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( keyfobapp_TaskID )) != NULL )
    {
      keyfobapp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & KFD_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &keyFob_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &keyFob_BondMgrCBs );

    // Start the Proximity Profile
    VOID ProxReporter_RegisterAppCBs( &keyFob_ProximityCBs );

    // Set timer for first battery read event
    osal_start_timerEx( keyfobapp_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );

    // Start the Accelerometer Profile
    VOID Accel_RegisterAppCBs( &keyFob_AccelCBs );

    //Set the proximity attribute values to default
    ProxReporter_SetParameter( PP_LINK_LOSS_ALERT_LEVEL,  sizeof ( uint8 ), &keyfobProxLLAlertLevel );
    ProxReporter_SetParameter( PP_IM_ALERT_LEVEL,  sizeof ( uint8 ), &keyfobProxIMAlertLevel );
    ProxReporter_SetParameter( PP_TX_POWER_LEVEL,  sizeof ( int8 ), &keyfobProxTxPwrLevel );

    // Set LED1 on to give feedback that the power is on, and a timer to turn off
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // To keep the LED on continuously.
    osal_start_timerEx( keyfobapp_TaskID, KFD_POWERON_LED_TIMEOUT_EVT, 1000 );
    
    return ( events ^ KFD_START_DEVICE_EVT );
  }

  if ( events & KFD_POWERON_LED_TIMEOUT_EVT )
  {
    osal_pwrmgr_device( PWRMGR_BATTERY ); // Revert to battery mode after LED off
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF ); 
    return ( events ^ KFD_POWERON_LED_TIMEOUT_EVT );
  }
  
  if ( events & KFD_ACCEL_READ_EVT )
  {
    bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

    if (status == SUCCESS)
    {
      if ( accelEnabler )
      {
        // Restart timer
        if ( ACCEL_READ_PERIOD )
        {
          osal_start_timerEx( keyfobapp_TaskID, KFD_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
        }

        // Read accelerometer data
        accelRead();
      }
      else
      {
        // Stop the acceleromter
        osal_stop_timerEx( keyfobapp_TaskID, KFD_ACCEL_READ_EVT);
      }
    }
    else
    {
        //??
    }
    return (events ^ KFD_ACCEL_READ_EVT);
  }

  if ( events & KFD_BATTERY_CHECK_EVT )
  {
    // Restart timer
    if ( BATTERY_CHECK_PERIOD )
    {
      osal_start_timerEx( keyfobapp_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    }

    // perform battery level check
    Batt_MeasLevel( );

    return (events ^ KFD_BATTERY_CHECK_EVT);
  }

  if ( events & KFD_TOGGLE_BUZZER_EVT )
  {
    // if this event was triggered while buzzer is on, turn it off, increment beep_count,
    // check whether max has been reached, and if not set the OSAL timer for next event to
    // turn buzzer back on.
    if ( buzzer_state == BUZZER_ON )
    {
      buzzerStop();
      buzzer_state = BUZZER_OFF;
      buzzer_beep_count++;
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY );
      #endif

      // check to see if buzzer has beeped maximum number of times
      // if it has, then don't turn it back on
      if ( ( buzzer_beep_count < BUZZER_MAX_BEEPS ) &&
           ( ( keyfobProximityState == KEYFOB_PROXSTATE_LINK_LOSS ) ||
             ( keyfobProximityState == KEYFOB_PROXSTATE_PATH_LOSS )    ) )
      {
        osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 800 );
      }
    }
    else if ( keyfobAlertState != ALERT_STATE_OFF )
    {
      // if this event was triggered while the buzzer is off then turn it on if appropriate
      keyfobapp_PerformAlert();
    }

    return (events ^ KFD_TOGGLE_BUZZER_EVT);
  }


#if defined ( PLUS_BROADCASTER )
  if ( events & KFD_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      keyfobapp_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      keyfobapp_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  }
}

/*********************************************************************
 * @fn      keyfobapp_HandleKeys
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
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;

    // if is active, pressing the left key should toggle
    // stop the alert
    if( keyfobAlertState != ALERT_STATE_OFF )
    {
      keyfobapp_StopAlert();
    }

    // if device is in a connection, toggle the Tx power level between 0 and
    // -6 dBm
    if( gapProfileState == GAPROLE_CONNECTED )
    {
      int8 currentTxPowerLevel;
      int8 newTxPowerLevel;

      ProxReporter_GetParameter( PP_TX_POWER_LEVEL, &currentTxPowerLevel );

      switch ( currentTxPowerLevel )
      {
      case 0:
        newTxPowerLevel = -6;
        // change power to -6 dBm
        HCI_EXT_SetTxPowerCmd( HCI_EXT_TX_POWER_MINUS_6_DBM );
        // Update Tx powerl level in Proximity Reporter (and send notification)
        // if enabled)
        ProxReporter_SetParameter( PP_TX_POWER_LEVEL, sizeof ( int8 ), &newTxPowerLevel );
        break;

      case (-6):
        newTxPowerLevel = 0;
        // change power to 0 dBm
        HCI_EXT_SetTxPowerCmd( HCI_EXT_TX_POWER_0_DBM );
        // Update Tx powerl level in Proximity Reporter (and send notification)
        // if enabled)
        ProxReporter_SetParameter( PP_TX_POWER_LEVEL, sizeof ( int8 ), &newTxPowerLevel );
        break;

      default:
        // do nothing
        break;
      }
    }

  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }

  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}

/*********************************************************************
 * @fn      keyfobapp_PerformAlert
 *
 * @brief   Performs an alert
 *
 * @param   none
 *
 * @return  none
 */
static void keyfobapp_PerformAlert( void )
{

  if ( keyfobProximityState == KEYFOB_PROXSTATE_LINK_LOSS )
  {
    switch( keyfobProxLLAlertLevel )
    {
    case PP_ALERT_LEVEL_LOW:

      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      #endif

      keyfobAlertState = ALERT_STATE_LOW;

      buzzerStart( BUZZER_ALERT_LOW_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
      break;

    case PP_ALERT_LEVEL_HIGH:

      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      #endif

      keyfobAlertState = ALERT_STATE_HIGH;

      buzzerStart( BUZZER_ALERT_HIGH_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
      HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );
      break;

    case PP_ALERT_LEVEL_NO:
        // Fall through
    default:
      keyfobapp_StopAlert();
      break;
    }
  }
  else if ( keyfobProximityState == KEYFOB_PROXSTATE_PATH_LOSS )
  {
    switch( keyfobProxIMAlertLevel )
    {
    case PP_ALERT_LEVEL_LOW:

      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      #endif

      keyfobAlertState = ALERT_STATE_LOW;

      buzzerStart( BUZZER_ALERT_LOW_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
      break;


    case PP_ALERT_LEVEL_HIGH:

      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      #endif

      keyfobAlertState = ALERT_STATE_HIGH;

      buzzerStart( BUZZER_ALERT_HIGH_FREQ );
      buzzer_state = BUZZER_ON;
      // only run buzzer for 200ms
      osal_start_timerEx( keyfobapp_TaskID, KFD_TOGGLE_BUZZER_EVT, 200 );

      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
      HalLedSet( HAL_LED_2, HAL_LED_MODE_FLASH );
      break;

      case PP_ALERT_LEVEL_NO:
        // Fall through
      default:
        keyfobapp_StopAlert();
        break;
      }
  }

}

/*********************************************************************
 * @fn      keyfobapp_StopAlert
 *
 * @brief   Stops an alert
 *
 * @param   none
 *
 * @return  none
 */
void keyfobapp_StopAlert( void )
{

  keyfobAlertState = ALERT_STATE_OFF;

  buzzerStop();
  buzzer_state = BUZZER_OFF;
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );


  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  uint16 connHandle = INVALID_CONNHANDLE;
  uint8 valFalse = FALSE;

  if ( gapProfileState != newState )
  {
    switch( newState )
    {
    case GAPROLE_STARTED:
      {
        // Set the system ID from the bd addr
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

        // shift three bytes up
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];

        // set middle bytes to zero
        systemId[4] = 0;
        systemId[3] = 0;

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    //if the state changed to connected, initially assume that keyfob is in range
    case GAPROLE_ADVERTISING:
      {
        // Visual feedback that we are advertising.
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
      }
      break;
      
    //if the state changed to connected, initially assume that keyfob is in range      
    case GAPROLE_CONNECTED:
      {
        // set the proximity state to either path loss alert or in range depending
        // on the value of keyfobProxIMAlertLevel (which was set by proximity monitor)
        if( keyfobProxIMAlertLevel != PP_ALERT_LEVEL_NO )
        {
          keyfobProximityState = KEYFOB_PROXSTATE_PATH_LOSS;
          // perform alert
          keyfobapp_PerformAlert();
          buzzer_beep_count = 0;
        }
        else // if keyfobProxIMAlertLevel == PP_ALERT_LEVEL_NO
        {
          keyfobProximityState = KEYFOB_PROXSTATE_CONNECTED_IN_RANGE;
          keyfobapp_StopAlert();
        }

        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &connHandle );

        #if defined ( PLUS_BROADCASTER )
          osal_start_timerEx( keyfobapp_TaskID, KFD_ADV_IN_CONNECTION_EVT, ADV_IN_CONN_WAIT );
        #endif
          
        // Turn off LED that shows we're advertising
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      }
      break;

    case GAPROLE_WAITING:
      {
        // then the link was terminated intentionally by the slave or master,
        // or advertising timed out
        keyfobProximityState = KEYFOB_PROXSTATE_INITIALIZED;

        // Turn off immediate alert
        ProxReporter_SetParameter(PP_IM_ALERT_LEVEL, sizeof(valFalse), &valFalse);
        keyfobProxIMAlertLevel = PP_ALERT_LEVEL_NO;
        
        // Change attribute value of Accelerometer Enable to FALSE
        Accel_SetParameter(ACCEL_ENABLER, sizeof(valFalse), &valFalse);
        // Stop the acceleromter
        accelEnablerChangeCB(); // SetParameter does not trigger the callback
        
        // Turn off LED that shows we're advertising
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
        
        // Stop alert if it was active
        if( keyfobAlertState != ALERT_STATE_OFF )
        {
          keyfobapp_StopAlert();
        }
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        // the link was dropped due to supervision timeout
        keyfobProximityState = KEYFOB_PROXSTATE_LINK_LOSS;

        // Turn off immediate alert
        ProxReporter_SetParameter(PP_IM_ALERT_LEVEL, sizeof(valFalse), &valFalse);
        keyfobProxIMAlertLevel = PP_ALERT_LEVEL_NO;
        
        // Change attribute value of Accelerometer Enable to FALSE
        Accel_SetParameter(ACCEL_ENABLER, sizeof(valFalse), &valFalse);
        // Stop the acceleromter
        accelEnablerChangeCB(); // SetParameter does not trigger the callback
        
        // Perform link loss alert if enabled
        if( keyfobProxLLAlertLevel != PP_ALERT_LEVEL_NO )
        {
          keyfobapp_PerformAlert();
          buzzer_beep_count = 0;
        }
      }
      break;

    default:
      // do nothing
      break;
    }
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      proximityAttrCB
 *
 * @brief   Notification from the profile of an atrribute change by
 *          a connected device.
 *
 * @param   attrParamID - Profile's Attribute Parameter ID
 *            PP_LINK_LOSS_ALERT_LEVEL  - The link loss alert level value
 *            PP_IM_ALERT_LEVEL  - The immediate alert level value
 *
 * @return  none
 */
static void proximityAttrCB( uint8 attrParamID )
{
  switch( attrParamID )
  {

  case PP_LINK_LOSS_ALERT_LEVEL:
    ProxReporter_GetParameter( PP_LINK_LOSS_ALERT_LEVEL, &keyfobProxLLAlertLevel );
    break;

  case PP_IM_ALERT_LEVEL:
    {
      ProxReporter_GetParameter( PP_IM_ALERT_LEVEL, &keyfobProxIMAlertLevel );

      // if proximity monitor set the immediate alert level to low or high, then
      // the monitor calculated that the path loss to the keyfob (proximity observer)
      // has exceeded the threshold
      if( keyfobProxIMAlertLevel != PP_ALERT_LEVEL_NO )
      {
        keyfobProximityState = KEYFOB_PROXSTATE_PATH_LOSS;
        keyfobapp_PerformAlert();
        buzzer_beep_count = 0;
      }
      else // proximity monitor turned off alert because the path loss is below threshold
      {
        keyfobProximityState = KEYFOB_PROXSTATE_CONNECTED_IN_RANGE;
        keyfobapp_StopAlert();
      }
    }
    break;

  default:
    // should not reach here!
    break;
  }

}

/*********************************************************************
 * @fn      accelEnablerChangeCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void accelEnablerChangeCB( void )
{
  bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

  if (status == SUCCESS){
    if (accelEnabler)
    {
      // Initialize accelerometer
      accInit();
      // Setup timer for accelerometer task
      osal_start_timerEx( keyfobapp_TaskID, KFD_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
    } else
    {
      // Stop the acceleromter
      accStop();
      osal_stop_timerEx( keyfobapp_TaskID, KFD_ACCEL_READ_EVT);
    }
  } else
  {
    //??
  }
}

/*********************************************************************
 * @fn      accelRead
 *
 * @brief   Called by the application to read accelerometer data
 *          and put data in accelerometer profile
 *
 * @param   none
 *
 * @return  none
 */
static void accelRead( void )
{

  static int8 x, y, z;
  int8 new_x, new_y, new_z;

  // Read data for each axis of the accelerometer
  accReadAcc(&new_x, &new_y, &new_z);

  // Check if x-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (x < (new_x-ACCEL_CHANGE_THRESHOLD)) || (x > (new_x+ACCEL_CHANGE_THRESHOLD)) )
  {
    x = new_x;
    Accel_SetParameter(ACCEL_X_ATTR, sizeof ( int8 ), &x);
  }

  // Check if y-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (y < (new_y-ACCEL_CHANGE_THRESHOLD)) || (y > (new_y+ACCEL_CHANGE_THRESHOLD)) )
  {
    y = new_y;
    Accel_SetParameter(ACCEL_Y_ATTR, sizeof ( int8 ), &y);
  }

  // Check if z-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (z < (new_z-ACCEL_CHANGE_THRESHOLD)) || (z > (new_z+ACCEL_CHANGE_THRESHOLD)) )
  {
    z = new_z;
    Accel_SetParameter(ACCEL_Z_ATTR, sizeof ( int8 ), &z);
  }

}

/*********************************************************************
*********************************************************************/
