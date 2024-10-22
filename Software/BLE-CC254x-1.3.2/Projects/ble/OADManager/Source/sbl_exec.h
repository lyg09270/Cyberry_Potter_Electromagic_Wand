/**************************************************************************************************
  Filename:       sbl_exec.h
  Revised:        $Date: 2012-11-16 18:39:26 -0800 (Fri, 16 Nov 2012) $
  Revision:       $Revision: 32218 $

  Description:

  Serial Bootloader Executive.


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
#ifndef SBL_EXEC_H
#define SBL_EXEC_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "comdef.h"
#include "hal_rpc.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// The SB page boundary since all SB addresses are "actual address / flash word size".
// Note for MSP - flash word size is 1, but 4 must be used for inter-compatibility w/ SoC.
#define SBL_PAGE_SIZE               (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)

#define SBL_FORCE_BOOT               0xF8
#define SBL_FORCE_RUN               (SBL_FORCE_BOOT ^ 0xFF)

#define SBL_RW_BUF_LEN               64

// Commands to Bootloader
#define SBL_WRITE_CMD                0x01
#define SBL_READ_CMD                 0x02
#define SBL_ENABLE_CMD               0x03
#define SBL_HANDSHAKE_CMD            0x04

// If an Application is to increase the number of NV pages used, the SBL AP can dynamically change
// the NV area "skipped" by the BootLoader. The payload is the Address to begin skipping / 4.
#define SBL_SKIP_ADDR_BEG_CMD        0x08
// If SBL_READ_CMD is disabled for security and the BootLoader must calculated the CRC, the code
// address end for the calculation with differ with the flash size. Instead of requiring a
// different BootLoader image for each flash size, and this flexibility to dynamically increase
// this code address end. This flexibility will also allow custom use of the end of the flash range
// for a 2nd, proprietary NV or other storage area.
#define SBL_CODE_ADDR_END_CMD        0x09

// Commands to Target Application
#define SBL_TGT_BOOTLOAD             0x10  // Erase the image valid signature & jump to bootloader.

// Responses from Bootloader - for historical consistency, SBL has OR'ed the MSBit of all commands
// when responding - this is probably not necessary for smooth functionality.
#define SBL_RSP_MASK                 0x80

// Status codes
#define SBL_SUCCESS                  0
#define SBL_FAILURE                  1
#define SBL_INVALID_FCS              2
#define SBL_INVALID_FILE             3
#define SBL_FILESYSTEM_ERROR         4
#define SBL_ALREADY_STARTED          5
#define SBL_NO_RESPOSNE              6
#define SBL_VALIDATE_FAILED          7
#define SBL_CANCELED                 8
#define SBL_IGNORED                  9

// Indices into the RPC data (RPC_POS_DAT0):
#define SBL_REQ_ADDR_LSB             RPC_POS_DAT0
#define SBL_REQ_ADDR_MSB            (SBL_REQ_ADDR_LSB+1)
#define SBL_REQ_DAT0                (SBL_REQ_ADDR_MSB+1)
#define SBL_RSP_STATUS               RPC_POS_DAT0
#define SBL_RSP_ADDR_LSB            (SBL_RSP_STATUS+1)
#define SBL_RSP_ADDR_MSB            (SBL_RSP_ADDR_LSB+1)
#define SBL_RSP_DAT0                (SBL_RSP_ADDR_MSB+1)
#define SBL_READ_HDR_LEN            (SBL_RSP_DAT0 - SBL_RSP_STATUS)

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          sblInit
 *
 * @brief       Boot Loader initialization.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if there is a valid RC image; FALSE otherwise.
 */
uint8 sblInit(void);

/**************************************************************************************************
 * @fn          sblExec
 *
 * @brief       Act on the SB command and received buffer.
 *
 * @param       pBuf - A pointer to the RPC command buffer received.
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void sblExec(uint8 *pBuf);

#endif
/**************************************************************************************************
*/
