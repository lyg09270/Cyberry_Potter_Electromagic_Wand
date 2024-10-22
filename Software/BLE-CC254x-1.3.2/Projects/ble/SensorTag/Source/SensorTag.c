/**************************************************************************************************
  Filename:       sensorTag.c
  Revised:        $Date: 2013-05-16 08:23:18 -0700 (Thu, 16 May 2013) $
  Revision:       $Revision: 34324 $

  Description:    This file contains the Sensor Tag sample application
                  for use with the TI Bluetooth Low Energy Protocol Stack.

  Copyright 2012-2013  Texas Instruments Incorporated. All rights reserved.

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
#include "hal_keys.h"
#include "hal_i2c.h"

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

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

// Services
#include "devinfoservice-st.h"
#include "irtempservice.h"
#include "accelerometerservice.h"
#include "humidityservice.h"
#include "magnetometerservice.h"
#include "barometerservice.h"
#include "gyroservice.h"
#include "testservice.h"
#include "simplekeys.h"

// Sensor drivers
#include "sensorTag.h"
#include "hal_sensor.h"

#include "hal_irtemp.h"
#include "hal_acc.h"
#include "hal_humi.h"
#include "hal_mag.h"
#include "hal_bar.h"
#include "hal_gyro.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define TEMP_DEFAULT_PERIOD                   1000
#define HUM_DEFAULT_PERIOD                    1000
#define BAR_DEFAULT_PERIOD                    1000
#define MAG_DEFAULT_PERIOD                    2000
#define ACC_DEFAULT_PERIOD                    1000
#define GYRO_DEFAULT_PERIOD                   1000

// Constants for two-stage reading
#define TEMP_MEAS_DELAY                       275   // Conversion time 250 ms
#define BAR_FSM_PERIOD                        80
#define ACC_FSM_PERIOD                        20
#define HUM_FSM_PERIOD                        20
#define GYRO_STARTUP_TIME                     60    // Start-up time max. 50 ms

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

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
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Side key bit
#define SK_KEY_SIDE                           0x04

// Test mode bit
#define TEST_MODE_ENABLE                      0x80

// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE                 0x00
#define ST_CFG_SENSOR_ENABLE                  0x01
#define ST_CFG_CALIBRATE                      0x02
#define ST_CFG_ERROR                          0xFF

// System reset
#define ST_SYS_RESET_DELAY                    3000

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
static uint8 sensorTag_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x65,   // 'e'
  0x6E,   // 'n'
  0x73,   // 's'
  0x6F,   // 'o'
  0x72,   // 'r'
  0x54,   // 'T'
  0x61,   // 'a'
  0x67,   // 'g'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100 ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
static uint8 attDeviceName[] = "TI BLE Sensor Tag";

// Sensor State Variables
static bool   irTempEnabled = FALSE;
static bool   magEnabled = FALSE;
static bool   accEnabled = FALSE;
static bool   barEnabled = FALSE;
static bool   humiEnabled = FALSE;
static bool   gyroEnabled = FALSE;

static bool   barBusy = FALSE;
static uint8  humiState = 0;

static bool   sysResetRequest = FALSE;

static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;
static uint16 sensorAccPeriod = ACC_DEFAULT_PERIOD;
static uint8  sensorGyroAxes = 0;
static bool   sensorGyroUpdateAxes = FALSE;
static uint16 selfTestResult = 0;
static bool   testMode = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

static void readIrTempData( void );
static void readHumData( void );
static void readAccData( void );
static void readMagData( void );
static void readBarData( void );
static void readBarCalibration( void );
static void readGyroData( void );

static void barometerChangeCB( uint8 paramID );
static void irTempChangeCB( uint8 paramID );
static void accelChangeCB( uint8 paramID );
static void humidityChangeCB( uint8 paramID);
static void magnetometerChangeCB( uint8 paramID );
static void gyroChangeCB( uint8 paramID );
static void testChangeCB( uint8 paramID );

static void resetSensorSetup( void );
static void sensorTag_HandleKeys( uint8 shift, uint8 keys );
static void resetCharacteristicValue(uint16 servID, uint8 paramID, uint8 value, uint8 paramLen);
static void resetCharacteristicValues();

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sensorTag_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sensorTag_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static barometerCBs_t sensorTag_BarometerCBs =
{
  barometerChangeCB,        // Characteristic value change callback
};

static irTempCBs_t sensorTag_IrTempCBs =
{
  irTempChangeCB,           // Characteristic value change callback
};

static accelCBs_t sensorTag_AccelCBs =
{
  accelChangeCB,            // Characteristic value change callback
};

static humidityCBs_t sensorTag_HumidCBs =
{
  humidityChangeCB,         // Characteristic value change callback
};

static magnetometerCBs_t sensorTag_MagnetometerCBs =
{
  magnetometerChangeCB,     // Characteristic value change callback
};

static gyroCBs_t sensorTag_GyroCBs =
{
  gyroChangeCB,             // Characteristic value change callback
};

static testCBs_t sensorTag_TestCBs =
{
  testChangeCB,             // Charactersitic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTag_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SensorTag_Init( uint8 task_id )
{
  sensorTag_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  // Setup the GAP Peripheral Role Profile
  {
    // Device starts advertising upon initialization
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
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }


  // Add services
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  IRTemp_AddService (GATT_ALL_SERVICES );         // IR Temperature Service
  Accel_AddService (GATT_ALL_SERVICES );          // Accelerometer Service
  Humidity_AddService (GATT_ALL_SERVICES );       // Humidity Service
  Magnetometer_AddService( GATT_ALL_SERVICES );   // Magnetometer Service
  Barometer_AddService( GATT_ALL_SERVICES );      // Barometer Service
  Gyro_AddService( GATT_ALL_SERVICES );           // Gyro Service
  SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
  Test_AddService( GATT_ALL_SERVICES );           // Test Profile

#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the Seensor Profile Characteristic Values
  resetCharacteristicValues();

  // Register for all key events - This app will handle all key events
  RegisterForKeys( sensorTag_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // Initialise sensor drivers
  HALIRTempInit();
  HalHumiInit();
  HalMagInit();
  HalAccInit();
  HalBarInit();
  HalGyroInit();

  // Register callbacks with profile
  VOID IRTemp_RegisterAppCBs( &sensorTag_IrTempCBs );
  VOID Magnetometer_RegisterAppCBs( &sensorTag_MagnetometerCBs );
  VOID Accel_RegisterAppCBs( &sensorTag_AccelCBs );
  VOID Humidity_RegisterAppCBs( &sensorTag_HumidCBs );
  VOID Barometer_RegisterAppCBs( &sensorTag_BarometerCBs );
  VOID Gyro_RegisterAppCBs( &sensorTag_GyroCBs );
  VOID Test_RegisterAppCBs( &sensorTag_TestCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( sensorTag_TaskID, ST_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SensorTag_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SensorTag_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sensorTag_TaskID )) != NULL )
    {
      sensorTag_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Handle system reset (long press on side key)
  if ( events & ST_SYS_RESET_EVT )
  {
    if (sysResetRequest)
    {
      HAL_SYSTEM_RESET();
    }
    return ( events ^ ST_SYS_RESET_EVT );
  }

  if ( events & ST_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &sensorTag_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &sensorTag_BondMgrCBs );

    return ( events ^ ST_START_DEVICE_EVT );
  }

  //////////////////////////
  //    IR TEMPERATURE    //
  //////////////////////////
  if ( events & ST_IRTEMPERATURE_READ_EVT )
  {
    if ( irTempEnabled )
    {
      if (HalIRTempStatus() == TMP006_DATA_READY)
      {
        readIrTempData();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, TEMP_DEFAULT_PERIOD-TEMP_MEAS_DELAY );
      }
      else if (HalIRTempStatus() == TMP006_OFF)
      {
        HalIRTempTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT, TEMP_MEAS_DELAY );
      }
    }
    else
    {
      //Turn off Temperatur sensor
      VOID HalIRTempTurnOff();
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,IRTEMPERATURE_DATA,0,IRTEMPERATURE_DATA_LEN);
      VOID resetCharacteristicValue(IRTEMPERATURE_SERV_UUID,IRTEMPERATURE_CONF,ST_CFG_SENSOR_DISABLE,sizeof ( uint8 ));
    }

    return (events ^ ST_IRTEMPERATURE_READ_EVT);
  }

  //////////////////////////
  //    Accelerometer     //
  //////////////////////////
  if ( events & ST_ACCELEROMETER_SENSOR_EVT )
  {
    if(accEnabled)
    {
      readAccData();
      osal_start_timerEx( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT, sensorAccPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_DATA, 0, ACCELEROMETER_DATA_LEN );
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_PERI, ACC_DEFAULT_PERIOD/ACCELEROMETER_TIME_UNIT, sizeof ( uint8 ));
    }

    return (events ^ ST_ACCELEROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Humidity        //
  //////////////////////////
  if ( events & ST_HUMIDITY_SENSOR_EVT )
  {
    if (humiEnabled)
    {
      HalHumiExecMeasurementStep(humiState);
      if (humiState == 2)
      {
        readHumData();
        humiState = 0;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, HUM_DEFAULT_PERIOD );
      }
      else
      {
        humiState++;
        osal_start_timerEx( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT, HUM_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_DATA, 0, HUMIDITY_DATA_LEN);
      resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ ST_HUMIDITY_SENSOR_EVT);
  }

  //////////////////////////
  //      Magnetometer    //
  //////////////////////////
  if ( events & ST_MAGNETOMETER_SENSOR_EVT )
  {
    if(magEnabled)
    {
      if (HalMagStatus() == MAG3110_DATA_READY)
      {
        readMagData();
      }
      else if (HalMagStatus() == MAG3110_OFF)
      {
        HalMagTurnOn();
      }

      osal_start_timerEx( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT, sensorMagPeriod );
    }
    else
    {
      HalMagTurnOff();
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_DATA, 0, MAGNETOMETER_DATA_LEN);
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_PERI, MAG_DEFAULT_PERIOD/MAGNETOMETER_TIME_UNIT,
                               sizeof ( uint8 ));
    }

    return (events ^ ST_MAGNETOMETER_SENSOR_EVT);
  }

  //////////////////////////
  //        Barometer     //
  //////////////////////////
  if ( events & ST_BAROMETER_SENSOR_EVT )
  {
    if (barEnabled)
    {
      if (barBusy)
      {
        barBusy = FALSE;
        readBarData();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, BAR_DEFAULT_PERIOD );
      }
      else
      {
        barBusy = TRUE;
        HalBarStartMeasurement();
        osal_start_timerEx( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT, BAR_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_DATA, 0, BAROMETER_DATA_LEN);
      resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_CALI, 0, BAROMETER_CALI_LEN);
    }

    return (events ^ ST_BAROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Gyroscope       //
  //////////////////////////
  if ( events & ST_GYROSCOPE_SENSOR_EVT )
  {
    uint8 status;

    status = HalGyroStatus();

    if(gyroEnabled)
    {
      if (status == HAL_GYRO_STOPPED)
      {
        HalGyroSelectAxes(sensorGyroAxes);
        HalGyroTurnOn();
        osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
      }
      else
      {
        if(sensorGyroUpdateAxes)
        {
          HalGyroSelectAxes(sensorGyroAxes);
          sensorGyroUpdateAxes = FALSE;
        }

        if (status == HAL_GYRO_DATA_READY)
        {
          readGyroData();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_DEFAULT_PERIOD - GYRO_STARTUP_TIME);
        }
        else
        {
          // Gyro needs to be activated;
          HalGyroWakeUp();
          osal_start_timerEx( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT, GYRO_STARTUP_TIME);
        }
      }
    }
    else
    {
      HalGyroTurnOff();
      if ( status == HAL_GYRO_STOPPED)
      {
        resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_DATA, 0, GYROSCOPE_DATA_LEN);
        resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
      }
      else
      {
        // Indicate error
        resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_DATA, ST_CFG_ERROR, GYROSCOPE_DATA_LEN);
        resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_CONF, ST_CFG_ERROR, sizeof( uint8 ));
      }
    }

    return (events ^ ST_GYROSCOPE_SENSOR_EVT);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & ST_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ ST_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      sensorTag_test
 *
 * @brief   Run a self-test of the sensor TAG
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 sensorTag_test(void)
{
  selfTestResult = HalSensorTest();
  HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);

  // Write the self-test result to the test service
  Test_SetParameter( TEST_DATA_ATTR, TEST_DATA_LEN, &selfTestResult);

  return selfTestResult;
}

/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      sensorTag_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void sensorTag_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      sensorTag_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      sensorTag_HandleKeys
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
static void sensorTag_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;  // Intentionally unreferenced parameter

  if (keys & HAL_KEY_SW_1)
  {
    // Reset the system if side key is pressed for more than 3 seconds
    sysResetRequest = TRUE;
    osal_start_timerEx( sensorTag_TaskID, ST_SYS_RESET_EVT, ST_SYS_RESET_DELAY );

    if (!testMode ) // Side key
    {
      // If device is not in a connection, pressing the side key should toggle
      //  advertising on and off
      if ( gapProfileState != GAPROLE_CONNECTED )
      {
        uint8 current_adv_enabled_status;
        uint8 new_adv_enabled_status;

        // Find the current GAP advertising status
        GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

        if( current_adv_enabled_status == FALSE )
        {
          new_adv_enabled_status = TRUE;
        }
        else
        {
          new_adv_enabled_status = FALSE;
        }

        // Change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }

      if ( gapProfileState == GAPROLE_CONNECTED )
      {
        uint8 adv_enabled = TRUE;

        // Disconnect
        GAPRole_TerminateConnection();
        // Start advertising
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enabled );
      }
    }
    else
    {
      // Test mode
      if ( keys & HAL_KEY_SW_1 ) // Side key
      {
        SK_Keys |= SK_KEY_SIDE;
      }
    }
  }

  if ( keys & HAL_KEY_SW_2 )   // Carbon S2
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_3 )   // Carbon S3
  {
    SK_Keys |= SK_KEY_RIGHT;
  }

  if (!(keys & HAL_KEY_SW_1))
  {
    // Cancel system reset request
    sysResetRequest = FALSE;
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}


/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
static void resetSensorSetup (void)
{
  if (HalIRTempStatus()!=TMP006_OFF || irTempEnabled)
  {
    HalIRTempTurnOff();
    irTempEnabled = FALSE;
  }

  if (accEnabled)
  {
    accEnabled = FALSE;
  }

  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }

  if (gyroEnabled)
  {
    HalGyroTurnOff();
    gyroEnabled = FALSE;
  }

  if (barEnabled)
  {
    HalBarInit();
    barEnabled = FALSE;
  }

  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }

  // Reset internal states
  sensorMagPeriod = MAG_DEFAULT_PERIOD;
  sensorAccPeriod = ACC_DEFAULT_PERIOD;
  sensorGyroAxes = 0;
  sensorGyroUpdateAxes = FALSE;
  testMode = FALSE;

  // Reset all characteristics values
  resetCharacteristicValues();
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
  switch ( newState )
  {
    case GAPROLE_STARTED:
    {
      uint8 ownAddress[B_ADDR_LEN];
      uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];

      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;

      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];

      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
    }
    break;

    case GAPROLE_ADVERTISING:
	    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );
	    break;

    case GAPROLE_CONNECTED:

      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );
      break;

    case GAPROLE_WAITING:
      // Link terminated intentionally: reset all sensors
      resetSensorSetup();
      break;

	  default:
	    break;
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      readAccData
 *
 * @brief   Read accelerometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readAccData(void)
{
  uint8 aData[ACCELEROMETER_DATA_LEN];

  if (HalAccRead(aData))
  {
    Accel_SetParameter( ACCELEROMETER_DATA, ACCELEROMETER_DATA_LEN, aData);
  }
}

/*********************************************************************
 * @fn      readMagData
 *
 * @brief   Read magnetometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readMagData( void )
{
  uint8 mData[MAGNETOMETER_DATA_LEN];

  if (HalMagRead(mData))
  {
    Magnetometer_SetParameter(MAGNETOMETER_DATA, MAGNETOMETER_DATA_LEN, mData);
  }
}

/*********************************************************************
 * @fn      readHumData
 *
 * @brief   Read humidity data
 *
 * @param   none
 *
 * @return  none
 */
static void readHumData(void)
{
  uint8 hData[HUMIDITY_DATA_LEN];

  if (HalHumiReadMeasurement(hData))
  {
    Humidity_SetParameter( HUMIDITY_DATA, HUMIDITY_DATA_LEN, hData);
  }
}

/*********************************************************************
 * @fn      readBarData
 *
 * @brief   Read barometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readBarData( void )
{
  uint8 bData[BAROMETER_DATA_LEN];

  if (HalBarReadMeasurement(bData))
  {
    Barometer_SetParameter( BAROMETER_DATA, BAROMETER_DATA_LEN, bData);
  }
}

/*********************************************************************
 * @fn      readBarCalibration
 *
 * @brief   Read barometer calibration
 *
 * @param   none
 *
 * @return  none
 */
static void readBarCalibration( void )
{
  uint8* cData = osal_mem_alloc(BAROMETER_CALI_LEN);

  if (cData != NULL )
  {
    HalBarReadCalibration(cData);
    Barometer_SetParameter( BAROMETER_CALI, BAROMETER_CALI_LEN, cData);
    osal_mem_free(cData);
  }
}

/*********************************************************************
 * @fn      readIrTempData
 *
 * @brief   Read IR temperature data
 *
 * @param   none
 *
 * @return  none
 */
static void readIrTempData( void )
{
  uint8 tData[IRTEMPERATURE_DATA_LEN];

  if (HalIRTempRead(tData))
  {
    IRTemp_SetParameter( IRTEMPERATURE_DATA, IRTEMPERATURE_DATA_LEN, tData);
  }
}

/*********************************************************************
 * @fn      readGyroData
 *
 * @brief   Read gyroscope data
 *
 * @param   none
 *
 * @return  none
 */
static void readGyroData( void )
{
  uint8 gData[GYROSCOPE_DATA_LEN];

  if (HalGyroRead(gData))
  {
    Gyro_SetParameter( GYROSCOPE_DATA, GYROSCOPE_DATA_LEN, gData);
  }
}

/*********************************************************************
 * @fn      barometerChangeCB
 *
 * @brief   Callback from Barometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void barometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case BAROMETER_CONF:
      Barometer_GetParameter( BAROMETER_CONF, &newValue );

      switch ( newValue)
      {
      case ST_CFG_SENSOR_DISABLE:
        if (barEnabled)
        {
          barEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_SENSOR_ENABLE:
        if(!barEnabled)
        {
          barEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_BAROMETER_SENSOR_EVT);
        }
        break;

      case ST_CFG_CALIBRATE:
        readBarCalibration();
        break;

      default:
        break;
      }
      break;

    default:
      // should not get here!
      break;
  }
}

/*********************************************************************
 * @fn      irTempChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void irTempChangeCB( uint8 paramID )
{
  if ( paramID == IRTEMPERATURE_CONF)
  {
    uint8 newValue;

    IRTemp_GetParameter( IRTEMPERATURE_CONF, &newValue );

    if ( newValue == ST_CFG_SENSOR_DISABLE)
    {
      // Put sensor to sleep
      if (irTempEnabled)
      {
        irTempEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_IRTEMPERATURE_READ_EVT);
      }
    }
    else if (newValue == ST_CFG_SENSOR_ENABLE)
    {
      if (!irTempEnabled)
      {
        irTempEnabled = TRUE;
        osal_set_event( sensorTag_TaskID,ST_IRTEMPERATURE_READ_EVT);
      }
    }
  }
}

/*********************************************************************
 * @fn      accelChangeCB
 *
 * @brief   Callback from Acceleromter Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void accelChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case ACCELEROMETER_CONF:
      Accel_GetParameter( ACCELEROMETER_CONF, &newValue );
      if ( newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (accEnabled)
        {
          accEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE)
      {
        if (!accEnabled)
        {
          accEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_ACCELEROMETER_SENSOR_EVT);
        }
      }
      break;

    case ACCELEROMETER_PERI:
      Accel_GetParameter( ACCELEROMETER_PERI, &newValue );
      sensorAccPeriod = newValue*ACCELEROMETER_TIME_UNIT;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      magnetometerChangeCB
 *
 * @brief   Callback from Magnetometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void magnetometerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case MAGNETOMETER_CONF:
      Magnetometer_GetParameter( MAGNETOMETER_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(magEnabled)
        {
          magEnabled = FALSE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!magEnabled)
        {
          magEnabled = TRUE;
          osal_set_event( sensorTag_TaskID, ST_MAGNETOMETER_SENSOR_EVT);
        }
      }
      break;

    case MAGNETOMETER_PERI:
      Magnetometer_GetParameter( MAGNETOMETER_PERI, &newValue );
      sensorMagPeriod = newValue*MAGNETOMETER_TIME_UNIT;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      humidityChangeCB
 *
 * @brief   Callback from Humidity Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void humidityChangeCB( uint8 paramID )
{
  if ( paramID == HUMIDITY_CONF)
  {
    uint8 newValue;

    Humidity_GetParameter( HUMIDITY_CONF, &newValue );

    if ( newValue == ST_CFG_SENSOR_DISABLE)
    {
      if (humiEnabled)
      {
        humiEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }

    if ( newValue == ST_CFG_SENSOR_ENABLE )
    {
      if (!humiEnabled)
      {
        humiEnabled = TRUE;
        humiState = 0;
        osal_set_event( sensorTag_TaskID, ST_HUMIDITY_SENSOR_EVT);
      }
    }
  }
}

/*********************************************************************
 * @fn      gyroChangeCB
 *
 * @brief   Callback from GyroProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void gyroChangeCB( uint8 paramID )
{
  if (paramID == GYROSCOPE_CONF)
  {
    uint8 newValue;

    Gyro_GetParameter( GYROSCOPE_CONF, &newValue );

    if (newValue == 0)
    {
      // All three axes off, put sensor to sleep
      if (gyroEnabled)
      {
        gyroEnabled = FALSE;
        osal_set_event( sensorTag_TaskID, ST_GYROSCOPE_SENSOR_EVT);
      }
    }
    else
    {
      // Bitmap tells which axis to enable (bit 0: X, but 1: Y, but 2: Z)
      gyroEnabled = TRUE;
      sensorGyroAxes = newValue & 0x07;
      sensorGyroUpdateAxes = TRUE;
      osal_set_event( sensorTag_TaskID,  ST_GYROSCOPE_SENSOR_EVT);
    }
  } // should not get here
}

/*********************************************************************
 * @fn      testChangeCB
 *
 * @brief   Callback from Test indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void testChangeCB( uint8 paramID )
{
  if( paramID == TEST_CONF_ATTR )
  {
    uint8 newValue;

    Test_GetParameter( TEST_CONF_ATTR, &newValue );

    if (newValue & TEST_MODE_ENABLE)
    {
      testMode = TRUE;
    }
    else
    {
      testMode = FALSE;
    }

    if (testMode)
    {
      // Test mode: possible to operate LEDs. Key hits will cause notifications,
      // side key does not influence connection state
      if (newValue & 0x01)
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x02)
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      }
    }
    else
    {
      // Normal mode; make sure LEDs are reset and attribute cleared
      HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      newValue = 0x00;
      Test_SetParameter( TEST_CONF_ATTR, 1, &newValue );
    }
  }
}


/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   vakue - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
    case IRTEMPERATURE_SERV_UUID:
      IRTemp_SetParameter( paramID, paramLen, pData);
      break;

    case ACCELEROMETER_SERV_UUID:
      Accel_SetParameter( paramID, paramLen, pData);
      break;

    case MAGNETOMETER_SERV_UUID:
      Magnetometer_SetParameter( paramID, paramLen, pData);
      break;

    case HUMIDITY_SERV_UUID:
      Humidity_SetParameter( paramID, paramLen, pData);
      break;

    case BAROMETER_SERV_UUID:
      Barometer_SetParameter( paramID, paramLen, pData);
      break;

    case GYROSCOPE_SERV_UUID:
      Gyro_SetParameter( paramID, paramLen, pData);
      break;

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values related to the sensors to zero
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, IRTEMPERATURE_DATA,0,IRTEMPERATURE_DATA_LEN);
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, IRTEMPERATURE_CONF,ST_CFG_SENSOR_DISABLE,sizeof ( uint8 ));

  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_DATA, 0, ACCELEROMETER_DATA_LEN );
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, ACCELEROMETER_PERI, ACC_DEFAULT_PERIOD/ACCELEROMETER_TIME_UNIT, sizeof ( uint8 ));

  resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_DATA, 0, HUMIDITY_DATA_LEN);
  resetCharacteristicValue( HUMIDITY_SERV_UUID, HUMIDITY_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));

  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_DATA, 0, MAGNETOMETER_DATA_LEN);
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( MAGNETOMETER_SERV_UUID, MAGNETOMETER_PERI,  MAG_DEFAULT_PERIOD/MAGNETOMETER_TIME_UNIT, sizeof ( uint8 ));

  resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_DATA, 0, BAROMETER_DATA_LEN);
  resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( BAROMETER_SERV_UUID, BAROMETER_CALI, 0, BAROMETER_CALI_LEN);

  resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_DATA, 0, GYROSCOPE_DATA_LEN);
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, GYROSCOPE_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
}


/*********************************************************************
*********************************************************************/

