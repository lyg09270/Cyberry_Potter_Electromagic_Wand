/*******************************************************************************
  Filename:       hci_c_data.h
  Revised:        $Date: 2011-08-22 08:41:40 -0700 (Mon, 22 Aug 2011) $
  Revision:       $Revision: 27235 $

  Description:    This file handles HCI data for the BLE Controller.

  Copyright 2009-2011 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
*******************************************************************************/

#ifndef HCI_C_DATA_H
#define HCI_C_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include "osal_bufmgr.h"

/*******************************************************************************
 * MACROS
 */

#define HCI_ResetControllerBuffers() HCI_TxDataBufferInit()

/*******************************************************************************
 * CONSTANTS
 */

#define UNDEFINED_CONN_HANDLE        0xFFFF

// Data State
#define DATA_BUF_FREE                0
#define DATA_BUF_IN_USE              1
#define DATA_BUF_PENDING             2

/*******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8  state;       // DATA_BUF_FREE, DATA_BUF_IN_USE, DATA_BUF_PENDING
  uint16 connHandle;  // Connection Handle
  uint8  fragFlag;    // Packet Boundary Flag
  uint16 len;         // Data Length
  uint8  *pData;      // Pointer to Packet Payload
} hciTxData_t;

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*
** HCI Data API
*/

/*******************************************************************************
 * This function will initialize the buffers for transmit data.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_TxDataBufferInit( void );


#if defined(CTRL_CONFIG) && ((CTRL_CONFIG & ADV_CONN_CFG) || (CTRL_CONFIG & INIT_CFG))
/*******************************************************************************
 * @fn          HCI_TxDataBufferInsert
 *
 * @brief       This function will insert a transmit data packet into the free
 *              buffers.
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       pbFlag     - Packet Boundary Flag.
 * @param       pktLen     - Number of bytes of data to transmit.
 * @param       *pData     - Pointer to data buffer to transmit.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      HCI_SUCCESS, HCI_ERROR_CODE_MEM_CAP_EXCEEDED
 */
extern hciStatus_t HCI_TxDataBufferInsert( uint16 connHandle,
                                           uint8  pbFlag,
                                           uint16 pktLen,
                                           uint8  *pData );
#endif // CTRL_CONFIG=(ADV_CONN_CFG | INIT_CFG)


#if defined(CTRL_CONFIG) && ((CTRL_CONFIG & ADV_CONN_CFG) || (CTRL_CONFIG & INIT_CFG))
/*******************************************************************************
 * @fn          HCI_TxDataSend
 *
 * @brief       This function sends an ACL transmit data packet to the LL. If
 *              the packet is successfully transferred to the TX FIFO by the
 *              LL, then the buffer can be freed. Otherwise, the packet is
 *              still pending in the LL, so it can't be released. If any error
 *              occurs (due to parametric checks), then the buffer is freed
 *              and a Number of Completed Packets event is generated with the
 *              number of completed packets set to zero.
 *
 * input parameters
 *
 * @param       connHandle - Connection handle, or HCI_TX_DATA_ANY_CONNECTION.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_TxDataSend( uint8 connHandle );
#endif // CTRL_CONFIG=(ADV_CONN_CFG | INIT_CFG)


/*******************************************************************************
 * @fn          HCI_ReverseBytes
 *
 * @brief       This function is used to reverse the order of the bytes in
 *              an array in place.
 *
 * input parameters
 *
 * @param       *buf - Pointer to buffer containing bytes to be reversed.
 * @param       len  - Number of bytes in buffer.
 *
 *              Note: The length must be even.
 *
 *              Note: The maximum length is 128 bytes.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_ReverseBytes( uint8 *buf,
                              uint8 len );


#ifdef __cplusplus
}
#endif

#endif /* HCI_C_DATA_H */
