/**************************************************************************************************
  Filename:       barometerservice.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    Barometer service


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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "barometerservice.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Barometer Service UUID
CONST uint8 barServUUID[TI_UUID_SIZE] =
{
  TI_UUID(BAROMETER_SERV_UUID),
};

// Barometer Characteristic value Data UUID
CONST uint8 barDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(BAROMETER_DATA_UUID),
};

// Barometer Characteristic value Configuration UUID
CONST uint8 barCalUUID[TI_UUID_SIZE] =
{
  TI_UUID(BAROMETER_CALI_UUID),
};

// Barometer Characteristic value Configuration UUID
CONST uint8 barCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(BAROMETER_CONF_UUID),
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static barometerCBs_t *barometer_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Barometer Profile Service attribute
static CONST gattAttrType_t barService = { TI_UUID_SIZE, barServUUID };

// Barometer Characteristic Properties
static uint8 barDataProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

static uint8 barData[BAROMETER_DATA_LEN] = { 0, 0, 0, 0};

// Barometer Data Characteristic Configuration
static gattCharCfg_t barDataConfig[GATT_MAX_NUM_CONN];

// Barometer Characteristic User Description
static uint8 barDataUserDesp[] = "Barometer Data";

// Barometer Characteristic Configuration Properties
static uint8 barCalProps = GATT_PROP_READ;

// Barometer Calibration Characteristic Configuration Value
static uint8 barCal[BAROMETER_CALI_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Barometer Calibration Characteristic Configuration
static gattCharCfg_t barCalConfig[GATT_MAX_NUM_CONN];

// Barometer Calibration Characteristic Configuration User Description
static uint8 barCalUserDesp[] = "Barometer Cali.";

// Barometer Characteristic Configuration Properties
static uint8 barCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Barometer Characteristic Configuration Value
static uint8 barCfg = 0;

// Barometer Characteristic Configuration User Description
static uint8 barCfgUserDesp[] = "Barometer Conf.";


/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t sensorBarometerAttrTbl[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&barService                      /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &barDataProps
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, barDataUUID },
        GATT_PERMIT_READ,
        0,
        barData
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)barDataConfig
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        barDataUserDesp
      },

    // Characteristic Config Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &barCfgProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, barCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &barCfg
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        barCfgUserDesp
      },


    // Characteristic Calibration Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &barCalProps
    },

      // Characteristic Value "Calibration"
      {
        { TI_UUID_SIZE, barCalUUID },
        GATT_PERMIT_READ,
        0,
        barCal
      },

      // Characteristic configuration "Calibration"
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)barCalConfig
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        barCalUserDesp
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 barometer_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t barometer_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );
static void barometer_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t sensorProfileCBs =
{
  barometer_ReadAttrCB,  // Read callback function pointer
  barometer_WriteAttrCB, // Write callback function pointer
  NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Barometer_AddService
 *
 * @brief   Initializes the Sensor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Barometer_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, barDataConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, barCalConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( barometer_HandleConnStatusCB );

  if ( services & BAROMETER_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( sensorBarometerAttrTbl,
                                          GATT_NUM_ATTRS( sensorBarometerAttrTbl ),
                                          &sensorProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      Barometer_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Barometer_RegisterAppCBs( barometerCBs_t *appCallbacks )
{
  if ( barometer_AppCBs == NULL )
  {
    if ( appCallbacks != NULL )
    {
      barometer_AppCBs = appCallbacks;
    }

    return ( SUCCESS );
  }

  return ( bleAlreadyInRequestedMode );
}

/*********************************************************************
 * @fn      Barometer_SetParameter
 *
 * @brief   Set a Barometer service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Barometer_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case BAROMETER_DATA:
      if ( len == BAROMETER_DATA_LEN )
      {
        VOID osal_memcpy( barData, value, BAROMETER_DATA_LEN );

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( barDataConfig, barData, FALSE,
                                   sensorBarometerAttrTbl, GATT_NUM_ATTRS( sensorBarometerAttrTbl ),
                                   INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BAROMETER_CALI:
      if ( len == BAROMETER_CALI_LEN )
      {
        VOID osal_memcpy( barCal, value, BAROMETER_CALI_LEN );

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( barCalConfig, barCal, FALSE,
                                   sensorBarometerAttrTbl, GATT_NUM_ATTRS( sensorBarometerAttrTbl ),
                                   INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BAROMETER_CONF:
      if ( len == sizeof ( uint8 ) )
      {
        barCfg = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Barometer_GetParameter
 *
 * @brief   Get a Barometer parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Barometer_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case BAROMETER_DATA:
      VOID osal_memcpy( value, barData, BAROMETER_DATA_LEN );
      break;

    case BAROMETER_CALI:
      VOID osal_memcpy( value, barCal, BAROMETER_CALI_LEN );
      break;

    case BAROMETER_CONF:
      *((uint8*)value) = barCfg;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          barometer_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 barometer_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  // 16-bit UUID
  switch ( uuid )
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case BAROMETER_DATA_UUID:
      *pLen = BAROMETER_DATA_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, BAROMETER_DATA_LEN );
      break;

    case BAROMETER_CALI_UUID:
      *pLen = BAROMETER_CALI_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, BAROMETER_CALI_LEN );
      break;

    case BAROMETER_CONF_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}

/*********************************************************************
* @fn      barometer_WriteAttrCB
*
* @brief   Validate attribute data prior to a write operation
*
* @param   connHandle - connection message was received on
* @param   pAttr - pointer to attribute
* @param   pValue - pointer to data to be written
* @param   len - length of data
* @param   offset - offset of the first octet to be written
*
* @return  Success or Failure
*/
static bStatus_t barometer_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint16 uuid;
  uint8 notifyApp = 0xFF;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch ( uuid )
  {
    case BAROMETER_DATA_UUID:
      //Should not get here
      break;

    case BAROMETER_CALI_UUID:
      //Should not get here
      break;

    case BAROMETER_CONF_UUID:
      //Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        *pCurValue = pValue[0];

        if( pAttr->pValue == &barCfg )
        {
          notifyApp = BAROMETER_CONF;
        }

      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY );
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && barometer_AppCBs && barometer_AppCBs->pfnBarometerChange )
  {
    barometer_AppCBs->pfnBarometerChange( notifyApp );
  }

  return ( status );
}

/*********************************************************************
 * @fn          barometer_HandleConnStatusCB
 *
 * @brief       Sensor Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void barometer_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
       GATTServApp_InitCharCfg( connHandle, barDataConfig );
    }
  }
}


/*********************************************************************
*********************************************************************/
