/**************************************************************************************************
  Filename:       bloodPressure.c

  Revised:        $Date: 2012-02-15 11:27:44 -0800 (Wed, 15 Feb 2012) $
  Revision:       $Revision: 29302 $

  Description:    This file contains the Bloodpressure sample application 
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
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_key.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "bpservice.h"
#include "devinfoservice.h"
#include "bloodPressure.h"
#include "timeapp.h"
#include "OSAL_Clock.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED


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

// Some values used to simulate measurements
#define FLAGS_IDX_MAX                         7      //3 flags c/f -- timestamp -- site

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

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

#define BP_DISCONNECT_PERIOD                  6000

#define CUFF_MAX                              40

#define TIMER_CUFF_PERIOD                     500

#define BP_STORE_MAX                          4     //max measurement storage count          


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 bloodPressureTaskId;

// Connection handle
uint16 gapConnHandle;

// Time stamp read from server
uint8 timeConfigDone;


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP State
static gaprole_States_t gapProfileState = GAPROLE_INIT;

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
static uint8 scanResponseData[] =
{
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   
  'B',
  'l',
  'o',
  'o',
  'd',
  'P',
  'r',
  'e',
  's',
  's',
  'u',
  'r',
  'e',
  ' ',
  'S',
  'e',
  'n',
  's',
  'o',
  'r',  
    // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),  
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),  
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL, //TX Power Level
  0       // 0dBm  
};

// Advertisement data
static uint8 advertData[] = 
{ 
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16( BLOODPRESSURE_SERV_UUID ),
  HI_UINT16( BLOODPRESSURE_SERV_UUID )
  /* Removed - No Target Address in AD or Scan Response - Single Bond
  0x07,
  0x17,
  0x15,0x53,0x00,0x57,0x60,0x00
  */
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BloodPressure Sensor";

// Bonded state
static bool timeAppBonded = FALSE;

// Bonded peer address
static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Last connection address
static uint8 lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};;

static bool connectedToLastAddress = false;


// GAP connection handle
static uint16 gapConnHandle;

//static gaprole_States_t gapRoleState = GAPROLE_INIT;



// BloodPressure measurement value stored in this structure
static attHandleValueInd_t  bloodPressureMeas;
static attHandleValueNoti_t bloodPressureIMeas;
static uint16 bpSystolic = 120; //mmg  
static uint16 bpDiastolic = 80; //mmg
static uint16 bpMAP = 90; //70-110mmg
static uint16 bpPulseRate = 60; //pulseRate
static uint8  bpUserId = 1;
static uint16 bpMeasStatus = 0;




// flags for simulated measurements
static const uint8 bloodPressureFlags[FLAGS_IDX_MAX] =
{
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP |BLOODPRESSURE_FLAGS_PULSE| BLOODPRESSURE_FLAGS_USER | BLOODPRESSURE_FLAGS_STATUS,
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_MMHG,
  BLOODPRESSURE_FLAGS_KPA,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP | BLOODPRESSURE_FLAGS_PULSE,
  0x00
};

// Program State
enum
{
  BPM_STATE_IDLE,            
  BPM_STATE_ADVERTISING,              
  BPM_STATE_CONNECTED             
};

// Measurement State
enum
{
  BPM_MEAS_STATE_IDLE,            
  BPM_MEAS_STATE_ACTIVE,             
  BPM_MEAS_STATE_READY            
};

// initial value of flags
static uint8 bloodPressureFlagsIdx = 0;

// cuff count, when we reach max, send final BP meas 
static uint8 cuffCount = 0;

static attHandleValueInd_t bpStoreMeas[10];
static uint8 bpStoreStartIndex =0;
static uint8 bpStoreIndex = 0;



/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void bloodPressureProcessGattMsg( gattMsgEvent_t *pMsg );
static void bloodPressure_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void bloodPressure_HandleKeys( uint8 shift, uint8 keys );
static void bpFinalMeas(void);
static void cuffMeas(void);
static void bpServiceCB(uint8 event);
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simulateMeas( void );
static void bpStoreIndications(attHandleValueInd_t* pInd);
static void bpSendStoredMeas();
static void updateUI( void );

#if (defined HAL_LCD) && (HAL_LCD == TRUE) 
static char *bdAddr2Str ( uint8 *pAddr );
#endif  

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bloodPressure_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                // When a valid RSSI is read from controller
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
 * @fn      BloodPressure_Init
 *
 * @brief   Initialization function for the BloodPressure App Task.
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
void BloodPressure_Init( uint8 task_id )
{
  bloodPressureTaskId = task_id;


    // Set for testing if chip default is 0xFFFFFF
    //uint8 slaveAddr[] = {0xff,0xff,0xff,0x3,0x3,0x3};
    //HCI_EXT_SetBDADDRCmd(slaveAddr);
 

  
  // Setup the GAP Peripheral Role Profile
  {
 
    // Press button to initiate adv and measurement
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
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanResponseData ), scanResponseData );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Stop config reads when done
  timeConfigDone = FALSE;
   
  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( bloodPressureTaskId );
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  BloodPressure_AddService( GATT_ALL_SERVICES );
  DevInfo_AddService( );

  // Register for BloodPressure service callback
  BloodPressure_Register( bpServiceCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( bloodPressureTaskId );
  
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
  osal_set_event( bloodPressureTaskId, BP_START_DEVICE_EVT );

  // Update UI
  updateUI();
   
}

/*********************************************************************
 * @fn      BloodPressure_ProcessEvent
 *
 * @brief   BloodPressure Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BloodPressure_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( bloodPressureTaskId )) != NULL )
    {
      bloodPressure_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & BP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bloodPressure_PeripheralCBs );
 
   // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &timeAppBondCB );

    updateUI();    
    
    return ( events ^ BP_START_DEVICE_EVT );
  }

  if ( events & BP_START_DISCOVERY_EVT )
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
    return ( events ^ BP_START_DISCOVERY_EVT );
  }
  
  ///////////////////////////////////
  // BP MEAS DONE
  ///////////////////////////////////
  if ( events & TIMER_BPMEAS_EVT )
  {
    // Perform final measurement
    bpFinalMeas();
    
    return (events ^ TIMER_BPMEAS_EVT);
  }

  ///////////////////////////////////
  // CUFF
  ///////////////////////////////////
  if ( events & BP_TIMER_CUFF_EVT )
  {
    // Perform a Cutoff Measurement
    cuffMeas(); 
    
    cuffCount++;
    
    // If cuff count not met, keep sending cuff measurements
    if(cuffCount < CUFF_MAX)
    {
      // Start interval timer to send BP, just for simulation
      osal_start_timerEx( bloodPressureTaskId, BP_TIMER_CUFF_EVT, TIMER_CUFF_PERIOD );
     
    }
    // Get ready to send final measurement
    else
    {
      // Start timer to send final BP meas
      osal_start_timerEx( bloodPressureTaskId, TIMER_BPMEAS_EVT, TIMER_CUFF_PERIOD );    
    }
      
    return (events ^ BP_TIMER_CUFF_EVT);
  }  

  // Enable Bloodpressure CCC
  if ( events & BP_CCC_UPDATE_EVT )
  {
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      
      //if previously connected and measurements are active send stored
      if( connectedToLastAddress == true)
      {
        //send stored measurements
        bpSendStoredMeas();
      }
    }      

    return (events ^ BP_CCC_UPDATE_EVT);
  }
  
  // Disconnect after sending measurement
  if ( events & BP_DISCONNECT_EVT )
  {
    
    uint8 advEnable = FALSE;
    
    //disable advertising on disconnect
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
    
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
    
    // Terminate Connection
    GAPRole_TerminateConnection();

    return (events ^ BP_DISCONNECT_EVT);
  }    
 
   return 0;
}

/*********************************************************************
 * @fn      bloodPressure_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void bloodPressure_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  case KEY_CHANGE:
      bloodPressure_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
 
  case GATT_MSG_EVENT:
      bloodPressureProcessGattMsg( (gattMsgEvent_t *) pMsg );
      break;
  default:
      break;
  }
}

/*********************************************************************
 * @fn      bloodPressure_HandleKeys
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
static void bloodPressure_HandleKeys( uint8 shift, uint8 keys )
{
 
  if ( keys & HAL_KEY_SW_1 )
  {
    // set simulated measurement flag index
    if (++bloodPressureFlagsIdx == FLAGS_IDX_MAX)
    {
      bloodPressureFlagsIdx = 0;
    }
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off and start a measurement
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;
      
      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
      
      if( current_adv_enabled_status == FALSE )
        new_adv_enabled_status = TRUE;
      else
        new_adv_enabled_status = FALSE;
      
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      
      //start simulation timer (start --> cuff -->measurement ready)
      osal_start_timerEx( bloodPressureTaskId, BP_TIMER_CUFF_EVT, TIMER_CUFF_PERIOD ); 
      
      //reset cuff count
      cuffCount = 0;
     
    }
    else //connected mode, simulate some measurements
    {
      simulateMeas();
    }      
  }
}

/*********************************************************************
 * @fn      bpFinalMeas
 *
 * @brief   Prepare and send a bloodPressure measurement indication
 *
 * @return  none
 */
static void bpFinalMeas(void)
{
    
  // att value notification structure 
  uint8 *p = bloodPressureMeas.value;
  
  //flags
  uint8 flags = bloodPressureFlags[bloodPressureFlagsIdx];
  
  // flags 1 byte long
  *p++ = flags;

  //bloodpressure components
  *p++ = bpSystolic;
  *p++;
  *p++ = bpDiastolic;
  *p++;
  *p++ = bpMAP;
  *p++;
  
  //timestamp
  if (flags & BLOODPRESSURE_FLAGS_TIMESTAMP)
  {
   
    UTCTimeStruct time;
  
    // Get time structure from OSAL
    osal_ConvertUTCTime( &time, osal_getClock() );
      
    *p++ = 0x07;
    *p++ = 0xdb;
    *p++=time.month;    
    *p++=time.day;  
    *p++=time.hour;    
    *p++=time.minutes;    
    *p++=time.seconds;   
  }
  
  if(flags & BLOODPRESSURE_FLAGS_PULSE)
  {
    *p++ =  bpPulseRate;
    *p++;    
  }
 
    if(flags & BLOODPRESSURE_FLAGS_USER)
  {
    *p++ =  bpUserId;    
  }

    if(flags & BLOODPRESSURE_FLAGS_STATUS)
  {
    *p++ =  bpMeasStatus;   
    *p++;
  }
  
  bloodPressureMeas.len = (uint8) (p - bloodPressureMeas.value);
  
  //store measurment
  bpStoreIndications(&bloodPressureMeas);   

  //send stored measurements
  bpSendStoredMeas();
  
  //start disconnect timer
  osal_start_timerEx( bloodPressureTaskId, BP_DISCONNECT_EVT, BP_DISCONNECT_PERIOD );    
  
}



/*********************************************************************
 * @fn      bpStoreIndications
 *
 * @brief   Queue indications
 *
 * @return  none
 */
static void bpStoreIndications(attHandleValueInd_t* pInd)
{

    //store measurement
    osal_memcpy(&bpStoreMeas[bpStoreIndex],pInd, ATT_MTU_SIZE);
    
    //store index
    bpStoreIndex = bpStoreIndex +1;
    if(bpStoreIndex > BP_STORE_MAX)
    {
      bpStoreIndex = 0;
    }
    
    if(bpStoreIndex == bpStoreStartIndex)
    {
      bpStoreStartIndex = bpStoreStartIndex +1;
      if(bpStoreStartIndex > BP_STORE_MAX)
      {
        bpStoreStartIndex = 0;
      }
    }  
}


/*********************************************************************
 * @fn      bpSendStoredMeas
 *
 * @brief   Send a stored measurement indication. An incoming indication 
 *          confirmation will trigger the next pending stored measurement.
 *
 * @return  none
 */
static void bpSendStoredMeas(void)
{
    bStatus_t status;  
  
  //we connected to this peer before so send any stored measurements
  if(bpStoreStartIndex != bpStoreIndex )
    {
     //send Measurement - can fail if not connected or CCC not enabled
     status  = BloodPressure_MeasIndicate( gapConnHandle, &bpStoreMeas[bpStoreStartIndex],  bloodPressureTaskId);
     
     // if sucess increment the counters, the indication confirmation will trigger
     // the next indication if there are more pending.
     if(status == SUCCESS)
     {
        bpStoreStartIndex = bpStoreStartIndex +1;
      
        // wrap around buffer 
        if(bpStoreStartIndex > BP_STORE_MAX)
        {
          bpStoreStartIndex = 0;
        }
     }
   }  
}






/*********************************************************************
 * @fn      timeAppProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void bloodPressureProcessGattMsg( gattMsgEvent_t *pMsg )
{
  
  //Measurement Indication Confirmation
  if( pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
      bpSendStoredMeas();
  }
  
  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    timeAppIndGattMsg( pMsg );
  }
  else if ( pMsg->method == ATT_READ_RSP ||
            pMsg->method == ATT_WRITE_RSP )
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
  
   
  // stop periodic measurement
  osal_stop_timerEx( bloodPressureTaskId, BP_TIMER_CUFF_EVT ); 
  

  // reset bloodPressure measurement client configuration
  uint16 param = 0;
  BloodPressure_SetParameter(BLOODPRESSURE_MEAS_CHAR_CFG, sizeof(uint16), (uint8 *) &param);

  // reset bloodPressure intermediate measurement client configuration
  BloodPressure_SetParameter(BLOODPRESSURE_IMEAS_CHAR_CFG, sizeof(uint16), (uint8 *) &param);

  
  uint8 advEnable = FALSE;
  
  //disable advertising on disconnect
  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
  
}

/*********************************************************************
 * @fn      gapProfileStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
 
  
  
  // if connected
  if ( newState == GAPROLE_CONNECTED )
  {
    linkDBItem_t  *pItem;

    
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    
    // Get connection handle
    GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );

    // Get peer bd address
    if ( (pItem = linkDB_Find( gapConnHandle )) != NULL)
    {
      // If connected to device without bond do service discovery
      if ( !osal_memcmp( pItem->addr, timeAppBondedAddr, B_ADDR_LEN ) )
      {
        timeAppDiscoveryCmpl = FALSE;
      }
      else
      {
        timeAppDiscoveryCmpl = TRUE;
      }
      
      // if this was last connection address don't do discovery
      if(osal_memcmp( pItem->addr, lastConnAddr, B_ADDR_LEN ))
      {
        timeAppDiscoveryCmpl = TRUE;
        connectedToLastAddress = true;
      }
      else
      {
        //save the last connected address  
        osal_memcpy(lastConnAddr, pItem->addr, B_ADDR_LEN );
      }

      // Initiate service discovery if necessary
      if ( timeAppDiscoveryCmpl == FALSE )
      {
       
       // osal_start_timerEx( bloodPressureTaskId, BP_START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
      }
        
    } 
  }
  // if disconnected
  else if ( gapProfileState == GAPROLE_CONNECTED && 
            newState != GAPROLE_CONNECTED )
  {
    timeAppDisconnected();
    
    //always stop intermediate timer
    osal_stop_timerEx( bloodPressureTaskId, BP_TIMER_CUFF_EVT ); 
    
  }    
  // if started
  else if ( newState == GAPROLE_STARTED )
  {
    // Initialize time clock display
    //timeAppClockInit();
  }
  
  gapProfileState = newState;
  
  updateUI();
  
}


/*********************************************************************
 * @fn      BP Service Callback
 *
 * @brief   Callback function for bloodpressure service.
 *
 * @param   event - service event
 *
 *   Need to know if BP Meas enabled so we can send stored meas.     
      
      #define BLOODPRESSURE_MEAS_NOTI_ENABLED         1
      #define BLOODPRESSURE_MEAS_NOTI_DISABLED        2

 * @return  none
 */
static void bpServiceCB(uint8 event)
{
 
  
  switch (event)
  {
  case BLOODPRESSURE_MEAS_NOTI_ENABLED:

    osal_set_event( bloodPressureTaskId, BP_CCC_UPDATE_EVT );
    
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
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    timeAppPairingStarted = TRUE;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    timeAppPairingStarted = FALSE;

    if ( status == SUCCESS )
    {
      linkDBItem_t  *pItem;
      
      if ( (pItem = linkDB_Find( gapConnHandle )) != NULL )
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
        osal_set_event( bloodPressureTaskId, BP_START_DISCOVERY_EVT );
      }
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
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, 0 );
}


/*********************************************************************
 * @fn      cuffMeas
 *
 * @brief   Prepare and send a bloodPressure measurement notification
 *
 * @return  none
 */
static void cuffMeas(void)
{
   
  
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_BLINK );
  HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );
  HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );
  
  // att value notification structure 
  uint8 *p = bloodPressureIMeas.value;
  
  //flags
  uint8 flags = BLOODPRESSURE_FLAGS_MMHG |BLOODPRESSURE_FLAGS_PULSE | BLOODPRESSURE_FLAGS_USER| BLOODPRESSURE_FLAGS_STATUS;
   
  // flags 1 byte long
  *p++ = flags;

  //bloodpressure components 
  // IEEE The 16–bit value contains a 4-bit exponent to base 10, 
  // followed by a 12-bit mantissa. Each is in twoscomplementform.
  *p++ = bpSystolic;  //120 = 0x0078  SFloat little endian = 0x7800 
  *p++;
  *p++ = 0xFF;   //not used in cuff  IEEE NaN =0x07FF
  *p++ = 0x07;
  *p++ = 0xFF;   //not used in cuff IEEE NaN =0x07FF
  *p++ = 0x07;
 
  if(flags & BLOODPRESSURE_FLAGS_PULSE)
  {
    *p++ =  bpPulseRate;
    *p++;    
  } 
  
  if(flags & BLOODPRESSURE_FLAGS_USER)
  {
    *p++ =  bpUserId;    
  }  
  
  if(flags & BLOODPRESSURE_FLAGS_STATUS)
  {
    *p++ =  bpMeasStatus;
    *p++;
  }
  
  bloodPressureIMeas.len = (uint8) (p - bloodPressureIMeas.value);
  
  //cuff measurements are not stored, only sent
  BloodPressure_IMeasNotify( gapConnHandle, &bloodPressureIMeas,  bloodPressureTaskId);

}


/*********************************************************************
 * @fn      simulateMeas
 *
 * @brief   Modify Measurement Values
 *
 * @param   none
 *
 * @return  none
 */
static void simulateMeas( void )
{
  
  if (gapProfileState == GAPROLE_CONNECTED)
  {

    if(bpSystolic < 150)
      bpSystolic +=1;
    else
      bpSystolic = 80;
    
    if(bpDiastolic < 110)
      bpDiastolic +=1;
    else
      bpDiastolic = 90;    

    if(bpMAP < 110)
      bpMAP +=1;
    else
      bpMAP =70;  
    
    if(bpPulseRate < 140)
      bpPulseRate +=1;
    else
      bpPulseRate =40;  
    
    if(bpUserId < 5)
      bpUserId +=1;
    else
      bpUserId =1;  
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
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
#endif 

/*********************************************************************
 * @fn      Update UI
 *
 * @brief   Update user interface LCD and LED
 *
 * @param   none
 *
 * @return  none
 */
static void updateUI( void )
{

  LCD_WRITE_STRING( "Blood Pressure",  HAL_LCD_LINE_1 );
  
  static uint8 ownAddress[B_ADDR_LEN];

 #if (defined HAL_LCD) && (HAL_LCD == TRUE) 
    //number of stored measuremnts
    uint16 count =0;
  
    //store index
    if( bpStoreIndex > bpStoreStartIndex )
    {
      count = bpStoreIndex - bpStoreStartIndex;
    }
    
    if( bpStoreStartIndex > bpStoreIndex )
    {
       count = ( BP_STORE_MAX-bpStoreStartIndex ) + bpStoreIndex+1;
    }
#endif
  
  //State
   switch (gapProfileState)
  {
    case GAPROLE_INIT:
          LCD_WRITE_STRING( "Initialized",  HAL_LCD_LINE_2 );
          break;
    case GAPROLE_STARTED: 
          LCD_WRITE_STRING( "Started",  HAL_LCD_LINE_2 );
          GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
          LCD_WRITE_STRING( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_3 );
          break;
    case GAPROLE_ADVERTISING:
         LCD_WRITE_STRING( "Advertising",  HAL_LCD_LINE_2 );
         LCD_WRITE_STRING( "                  ",  HAL_LCD_LINE_3 );
         LCD_WRITE_STRING_VALUE("Stored", count, 10, HAL_LCD_LINE_3 );
         break;
    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:
         LCD_WRITE_STRING( "Waiting    ",  HAL_LCD_LINE_2 );
         LCD_WRITE_STRING_VALUE("Stored", count, 10, HAL_LCD_LINE_3 );  
        break;
    case GAPROLE_CONNECTED:
        LCD_WRITE_STRING( "Connected  ",  HAL_LCD_LINE_2 );
        break;
    case GAPROLE_ERROR:
        LCD_WRITE_STRING( "Error      ",  HAL_LCD_LINE_2 );
        break;
    default:
        break;
  }
  
}

/*********************************************************************
*********************************************************************/
