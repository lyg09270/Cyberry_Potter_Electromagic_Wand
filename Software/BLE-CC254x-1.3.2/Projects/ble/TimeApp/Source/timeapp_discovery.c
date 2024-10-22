/**************************************************************************************************
  Filename:       timeapp_discovery.c
  Revised:        $Date: 2011-12-16 15:46:52 -0800 (Fri, 16 Dec 2011) $
  Revision:       $Revision: 58 $

  Description:    Time App service and characteristic discovery routines.

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
#include "hal_lcd.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "timeapp.h"
#include "battservice.h"

/*********************************************************************
 * MACROS
 */

// Characteristic UUIDs
#define CT_TIME_UUID                    0x2A2B    // Time
#define LOCAL_TIME_INFO_UUID            0x2A0F    // Local Time Information
#define TIME_WITH_DST_UUID              0x2A11    // Time with DST
#define REF_TIME_INFO_UUID              0x2A14    // Reference Time Information
#define TIME_UPDATE_CTRL_UUID           0x2A16    // Time Update Control Point
#define TIME_UPDATE_STATE_UUID          0x2A17    // Time Update State
#define NETWORK_AVAIL_UUID              0x2A3E    // Network Availability
#define ALERT_STATUS_UUID               0x2A3F    // Alert Status
#define RINGER_CP_UUID                  0x2A40    // Ringer Control Point
#define RINGER_SETTING_UUID             0x2A41    // Ringer Setting
#define ALERT_NOTIF_CP_UUID             0x2A44    // Alert Notification Control Point
#define UNREAD_ALERT_STATUS_UUID        0x2A45    // Unread Alert Status
#define NEW_ALERT_UUID                  0x2A46    // New Alert
#define SUP_NEW_ALERT_CAT_UUID          0x2A47    // Supported New Alert Category
#define SUP_UNREAD_ALERT_CAT_UUID       0x2A48    // Supported Unread Alert Category

// Length of Characteristic declaration + handle with 16 bit UUID
#define CHAR_DESC_HDL_UUID16_LEN        7

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

// Attribute handle cache
uint16 timeAppHdlCache[HDL_CACHE_LEN];

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Attribute handles used during discovery
static uint16 timeAppSvcStartHdl;
static uint16 timeAppSvcEndHdl;
static uint8 timeAppEndHdlIdx;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 TimeAppDiscCurrTime( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscDstChg( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscRefTime( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscNwa( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscBatt( uint8 state, gattMsgEvent_t *pMsg );
static uint8 TimeAppDiscPhoneAlert( uint8 state, gattMsgEvent_t *pMsg );

/*********************************************************************
 * @fn      timeAppDiscStart()
 *
 * @brief   Start service discovery. 
 *
 *
 * @return  New discovery state.
 */
uint8 timeAppDiscStart( void )
{
  // Clear handle cache
  osal_memset( timeAppHdlCache, 0, sizeof(timeAppHdlCache) );
  
  // Start discovery with first service
  return timeAppDiscGattMsg( DISC_CURR_TIME_START, NULL );
}

/*********************************************************************
 * @fn      timeAppDiscGattMsg()
 *
 * @brief   Handle GATT messages for characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
uint8 timeAppDiscGattMsg( uint8 state, gattMsgEvent_t *pMsg )
{
  // Execute discovery function for service
  do
  {
    switch ( state & 0xF0 )
    {
      // Current time service
      case DISC_CURR_TIME_START:
        state = TimeAppDiscCurrTime( state, pMsg );
        if ( state == DISC_FAILED )
        {
          state = DISC_NWA_START;
        }
        else if ( state == DISC_IDLE )
        {
          state = DISC_DST_CHG_START;
        }
        break;

      // DST change service
      case DISC_DST_CHG_START:
        state = TimeAppDiscDstChg( state, pMsg );
        if ( state == DISC_FAILED )
        {
          state = DISC_NWA_START;
        }
        else if ( state == DISC_IDLE )
        {
          state = DISC_REF_TIME_START;
        }
        break;

      // Reference time service
      case DISC_REF_TIME_START:
        state = TimeAppDiscRefTime( state, pMsg );
        if ( state == DISC_FAILED || state == DISC_IDLE )
        {
          state = DISC_NWA_START;
        }
        break;

      // NwA service
      case DISC_NWA_START:
        state = TimeAppDiscNwa( state, pMsg );
        if ( state == DISC_FAILED || state == DISC_IDLE )
        {
          state = DISC_ALERT_NTF_START;
        }
        break;

      // Alert notification service
      case DISC_ALERT_NTF_START:
        state = TimeAppDiscAlertNtf( state, pMsg );
        if ( state == DISC_FAILED || state == DISC_IDLE )
        {
          state = DISC_BATT_START;
        }
        break;

      // Battery service
      case DISC_BATT_START:
        state = TimeAppDiscBatt( state, pMsg );
        if ( state == DISC_FAILED || state == DISC_IDLE )
        {
          state = DISC_PAS_START;
        }
        break;

      // Phone alert status service
      case DISC_PAS_START:
        state = TimeAppDiscPhoneAlert( state, pMsg );
        if ( state == DISC_FAILED )
        {
          state = DISC_IDLE;
        }
        break;

      default:
        break;
    }
  } while ( (state != 0) && ((state & 0x0F) == 0) );
  
  return state;    
}

/*********************************************************************
 * @fn      TimeAppDiscCurrTime()
 *
 * @brief   Current time service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscCurrTime( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_CURR_TIME_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(CURRENT_TIME_SVC_UUID),
                                         HI_UINT16(CURRENT_TIME_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_CURR_TIME_SVC;
      } 
      break;

    case DISC_CURR_TIME_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_CURR_TIME_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_CURR_TIME_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case CT_TIME_UUID:
                timeAppHdlCache[HDL_CURR_TIME_CT_TIME_START] = handle;
                timeAppEndHdlIdx = HDL_CURR_TIME_CT_TIME_END;
                break;
                
              case LOCAL_TIME_INFO_UUID:
                timeAppHdlCache[HDL_CURR_TIME_LOC_INFO] = handle;
                break;
                
              case REF_TIME_INFO_UUID:
                timeAppHdlCache[HDL_CURR_TIME_REF_INFO] = handle;
                break;
                
              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find time characteristic
          if ( timeAppHdlCache[HDL_CURR_TIME_CT_TIME_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_CURR_TIME_CT_TIME_START] <
                   timeAppHdlCache[HDL_CURR_TIME_CT_TIME_END] )
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_CURR_TIME_CT_TIME_START] + 1,
                                   timeAppHdlCache[HDL_CURR_TIME_CT_TIME_END],
                                   timeAppTaskId );
                                        
            newState = DISC_CURR_TIME_CT_TIME_CCCD;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;

    case DISC_CURR_TIME_CT_TIME_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_CURR_TIME_CT_TIME_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscDstChg()
 *
 * @brief   DST change service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscDstChg( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_DST_CHG_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(DST_CHANGE_SVC_UUID),
                                         HI_UINT16(DST_CHANGE_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_DST_CHG_SVC;
      } 
      break;

    case DISC_DST_CHG_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_DST_CHG_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_DST_CHG_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case TIME_WITH_DST_UUID:
                timeAppHdlCache[HDL_DST_CHG_TIME_DST] = handle;
                break;
                
              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // If didn't find DST characteristic
          if ( timeAppHdlCache[HDL_DST_CHG_TIME_DST] == 0 )
          {
            newState = DISC_FAILED;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;

    default:
      break;
  }

  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscRefTime()
 *
 * @brief   Reference time service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscRefTime( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_REF_TIME_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(REF_TIME_UPDATE_SVC_UUID),
                                         HI_UINT16(REF_TIME_UPDATE_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_REF_TIME_SVC;
      } 
      break;

    case DISC_REF_TIME_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_REF_TIME_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_REF_TIME_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case TIME_UPDATE_CTRL_UUID:
                timeAppHdlCache[HDL_REF_TIME_UPD_CTRL] = handle;
                break;
                
              case TIME_UPDATE_STATE_UUID:
                timeAppHdlCache[HDL_REF_TIME_UPD_STATE] = handle;
                break;
                
              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // If didn't find mandatory characteristics
          if ( timeAppHdlCache[HDL_REF_TIME_UPD_CTRL] == 0 ||
               timeAppHdlCache[HDL_REF_TIME_UPD_STATE] == 0 )
          {
            newState = DISC_FAILED;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;

    default:
      break;
  }

  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscNwa()
 *
 * @brief   NwA service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscNwa( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_NWA_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(NWA_SVC_UUID),
                                         HI_UINT16(NWA_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_NWA_SVC;
      } 
      break;

    case DISC_NWA_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_NWA_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_NWA_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN)
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case NETWORK_AVAIL_UUID:
                timeAppHdlCache[HDL_NWA_NWA_START] = handle;
                timeAppEndHdlIdx = HDL_NWA_NWA_END;
                break;
                
              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find network availability characteristic
          if ( timeAppHdlCache[HDL_NWA_NWA_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_NWA_NWA_START] <
                    timeAppHdlCache[HDL_NWA_NWA_END] )
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_NWA_NWA_START] + 1,
                                   timeAppHdlCache[HDL_NWA_NWA_END],
                                   timeAppTaskId );
                                        
            newState = DISC_NWA_NWA_CCCD;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;

    case DISC_NWA_NWA_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_NWA_NWA_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscAlertNtf()
 *
 * @brief   Alert notification service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_ALERT_NTF_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(ALERT_NOTIF_SVC_UUID),
                                         HI_UINT16(ALERT_NOTIF_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_ALERT_NTF_SVC;
      } 
      break;

    case DISC_ALERT_NTF_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_ALERT_NTF_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_ALERT_NTF_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN )
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case ALERT_NOTIF_CP_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_CTRL] = handle;
                break;

              case UNREAD_ALERT_STATUS_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] = handle;
                timeAppEndHdlIdx = HDL_ALERT_NTF_UNREAD_END;
                break;

              case NEW_ALERT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = handle;
                timeAppEndHdlIdx = HDL_ALERT_NTF_NEW_END;
                break;

              case SUP_NEW_ALERT_CAT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_NEW_CAT] = handle;
                break;

              case SUP_UNREAD_ALERT_CAT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_UNREAD_CAT] = handle;
                break;

              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find new alert characteristic
          if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_START] <
                    timeAppHdlCache[HDL_ALERT_NTF_NEW_END] )
          {
            // Discover incoming alert characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_ALERT_NTF_NEW_START] + 1,
                                   timeAppHdlCache[HDL_ALERT_NTF_NEW_END],
                                   timeAppTaskId );
                                        
            newState = DISC_ALERT_NTF_NEW_CCCD;
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = 0;
            newState = DISC_FAILED;
          }
        }
      }      
      break;

    case DISC_ALERT_NTF_NEW_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_ALERT_NTF_NEW_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // If CCCD found
          if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_CCCD] != 0 )
          {
            // Should we look for unread category status CCCD
            if ( timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] <
                 timeAppHdlCache[HDL_ALERT_NTF_UNREAD_END] )
            {
              // Discover unread category status characteristic descriptors
              GATT_DiscAllCharDescs( timeAppConnHandle,
                                     timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] + 1,
                                     timeAppHdlCache[HDL_ALERT_NTF_UNREAD_END],
                                     timeAppTaskId );
                                          
              newState = DISC_ALERT_NTF_UNREAD_CCCD;
            }
            else
            {
              // Done
              newState = DISC_IDLE;
            }
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = 0;
            newState = DISC_FAILED;
          }          
        }
      }
      break;

    case DISC_ALERT_NTF_UNREAD_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_ALERT_NTF_UNREAD_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscBatt()
 *
 * @brief   Battery service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscBatt( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
    case DISC_BATT_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(BATT_SERVICE_UUID),
                                         HI_UINT16(BATT_SERVICE_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_BATT_SVC;
      } 
      break;

    case DISC_BATT_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_BATT_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_BATT_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN )
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case BATT_LEVEL_UUID:
                timeAppHdlCache[HDL_BATT_LEVEL_START] = handle;
                timeAppEndHdlIdx = HDL_BATT_LEVEL_END;
                break;
                                
              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }
          
        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find mandatory characteristic
          if ( timeAppHdlCache[HDL_BATT_LEVEL_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_BATT_LEVEL_START] <
                    timeAppHdlCache[HDL_BATT_LEVEL_END] )
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_BATT_LEVEL_START] + 1,
                                   timeAppHdlCache[HDL_BATT_LEVEL_END],
                                   timeAppTaskId );
                                        
            newState = DISC_BATT_LVL_CCCD;
          }
          else
          {
            newState = DISC_IDLE;
          }
        }
      }      
      break;

    case DISC_BATT_LVL_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_BATT_LEVEL_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          newState = DISC_IDLE;
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
 * @fn      TimeAppDiscPhoneAlert()
 *
 * @brief   Phone alert status service and characteristic discovery. 
 *
 * @param   state - Discovery state.
 * @param   pMsg - GATT message.
 *
 * @return  New discovery state.
 */
static uint8 TimeAppDiscPhoneAlert( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;

  switch ( state )
  {
    case DISC_PAS_START:  
      {
        uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(PHONE_ALERT_STS_SVC_UUID),
                                         HI_UINT16(PHONE_ALERT_STS_SVC_UUID) };

        // Initialize service discovery variables
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_PAS_SVC;
      } 
      break;

    case DISC_PAS_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
          newState = DISC_PAS_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }    
      break;

    case DISC_PAS_CHAR:
      {
        uint8   i;
        uint8   *p;
        uint16  handle;
        uint16  uuid;
        
        // Characteristics found
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
             pMsg->msg.readByTypeRsp.numPairs > 0 && 
             pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN )
        {
          // For each characteristic declaration
          p = pMsg->msg.readByTypeRsp.dataList;
          for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
          {
            // Parse characteristic declaration
            handle = BUILD_UINT16(p[3], p[4]);
            uuid = BUILD_UINT16(p[5], p[6]);
                   
            // If looking for end handle
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case RINGER_CP_UUID:
                timeAppHdlCache[HDL_PAS_CTRL] = handle;
                break;

              case ALERT_STATUS_UUID:
                timeAppHdlCache[HDL_PAS_ALERT_START] = handle;
                timeAppEndHdlIdx = HDL_PAS_ALERT_END;
                break;

              case RINGER_SETTING_UUID:
                timeAppHdlCache[HDL_PAS_RINGER_START] = handle;
                timeAppEndHdlIdx = HDL_PAS_RINGER_END;
                break;

              default:
                break;
            }
            
            p += CHAR_DESC_HDL_UUID16_LEN;
          }
          
        }

        // If procedure complete
        if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // Special case of end handle at end of service
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find alert status characteristic
          if ( timeAppHdlCache[HDL_PAS_ALERT_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_PAS_ALERT_START] <
                    timeAppHdlCache[HDL_PAS_ALERT_END] )
          {
            // Discover alert status characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_PAS_ALERT_START] + 1,
                                   timeAppHdlCache[HDL_PAS_ALERT_END],
                                   timeAppTaskId );
                                        
            newState = DISC_PAS_ALERT_CCCD;
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_PAS_ALERT_START] = 0;
            newState = DISC_FAILED;
          }
        }
      }      
      break;

    case DISC_PAS_ALERT_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_PAS_ALERT_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // If CCCD found
          if ( timeAppHdlCache[HDL_PAS_ALERT_CCCD] != 0 )
          {
            // Should we look for ringer status CCCD
            if ( timeAppHdlCache[HDL_PAS_RINGER_START] <
                 timeAppHdlCache[HDL_PAS_RINGER_END] )
            {
              // Discover ringer status characteristic descriptors
              GATT_DiscAllCharDescs( timeAppConnHandle,
                                     timeAppHdlCache[HDL_PAS_RINGER_START] + 1,
                                     timeAppHdlCache[HDL_PAS_RINGER_END],
                                     timeAppTaskId );
                                          
              newState = DISC_PAS_RINGER_CCCD;
            }
            else
            {
              // Missing required characteristic descriptor
              timeAppHdlCache[HDL_PAS_RINGER_START] = 0;
              newState = DISC_FAILED;
            }
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_PAS_ALERT_START] = 0;
            newState = DISC_FAILED;
          }          
        }
      }
      break;

    case DISC_PAS_RINGER_CCCD:
      {
        uint8 i;
        
        // Characteristic descriptors found
        if ( pMsg->method == ATT_FIND_INFO_RSP &&
             pMsg->msg.findInfoRsp.numInfo > 0 && 
             pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
        {
          // For each handle/uuid pair
          for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
          {
            // Look for CCCD
            if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                  LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
                 (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
                  HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
            {
              // CCCD found
              timeAppHdlCache[HDL_PAS_RINGER_CCCD] =
                pMsg->msg.findInfoRsp.info.btPair[i].handle;
              
              break;
            }
          }
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
               pMsg->hdr.status == bleProcedureComplete ) ||
             ( pMsg->method == ATT_ERROR_RSP ) )
        {
          // If CCCD found
          if ( timeAppHdlCache[HDL_PAS_RINGER_CCCD] != 0 )
          {
            newState = DISC_IDLE;
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_PAS_RINGER_START] = 0;
            newState = DISC_FAILED;
          }
        }
      }
      break;

    default:
      break;
  }
  
  return newState;
}

/*********************************************************************
*********************************************************************/
