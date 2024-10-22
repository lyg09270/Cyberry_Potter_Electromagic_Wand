/**************************************************************************************************
  Filename:       magnetometerservice.h
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    Magnetometer service definitions and prototypes

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

#ifndef MAGNETOMETERSERVICE_H
#define MAGNETOMETERSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define MAGNETOMETER_DATA               7       // R  uint8 - Profile Attribute value
#define MAGNETOMETER_CONF               8       // RW uint8 - Profile Attribute value
#define MAGNETOMETER_PERI               9       // RW uint8 - Profile Attribute value

// Service UUID
#define MAGNETOMETER_SERV_UUID          0xAA30  // F0000000-0451-4000-B000-00000000-AA30
#define MAGNETOMETER_DATA_UUID          0xAA31
#define MAGNETOMETER_CONF_UUID          0xAA32
#define MAGNETOMETER_PERI_UUID          0xAA33

// Sensor Profile Services bit fields
#define MAGNETOMETER_SERVICE            0x00000008

// Length of sensor data in bytes
#define MAGNETOMETER_DATA_LEN           6

// Lower limits
#define MAGNETOMETER_PERIOD_MIN         100
#define MAGNETOMETER_TIME_UNIT          10 // resolution 10 ms


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef NULL_OK void (*magnetometerChange_t)( uint8 paramID );

typedef struct
{
  magnetometerChange_t        pfnMagnetometerChange;  // Called when characteristic value changes
} magnetometerCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * Magnetometer_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Magnetometer_AddService( uint32 services );

/*
 * Magnetometer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Magnetometer_RegisterAppCBs( magnetometerCBs_t *appCallbacks );

/*
 * Magnetometer_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Magnetometer_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Magnetometer_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Magnetometer_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MAGNETOMETERSERVICE_H */
