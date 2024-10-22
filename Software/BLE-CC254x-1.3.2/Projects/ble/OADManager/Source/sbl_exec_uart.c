/**************************************************************************************************
  Filename:       sbl_uart_exec.c
  Revised:        $Date: 2012-11-16 18:39:26 -0800 (Fri, 16 Nov 2012) $
  Revision:       $Revision: 32218 $

  Description:

  This file contains the interface to the H/W transport driver by UART which act as a
  Serial Boot Loader in order to get an OAD image into the OAD Dongle's internal flash.


  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_rpc.h"
#include "hal_uart.h"
#include "oad.h"
#include "sbl_exec.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// Serial RX States
#define SBL_SOF_STATE                0
#define SBL_LEN_STATE                1
#define SBL_CMD1_STATE               2
#define SBL_CMD2_STATE               3
#define SBL_DATA_STATE               4
#define SBL_FCS_STATE                5

// Buffer size - it has to be big enough for the largest NPI RPC packet and NPI overhead.
#define SBL_BUF_SIZE                 256
#define SBL_MAX_SIZE                (SBL_BUF_SIZE - SBL_FCS_STATE)

#define OAD_DONGLE_PAGE_MULT     ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))
#define OAD_DONGLE_SBL_IMG_MAX   64
#define OAD_DONGLE_SBL_IMG_BEG  (OAD_IMG_B_PAGE * OAD_DONGLE_PAGE_MULT)
#define OAD_DONGLE_SBL_IMG_END  (OAD_DONGLE_SBL_IMG_BEG + \
                                 OAD_DONGLE_SBL_IMG_MAX * OAD_DONGLE_PAGE_MULT)

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 sbBuf[SBL_BUF_SIZE], sbFcs, sbIdx, sbLen, sbSte;
static uint8 txIdx, txLen;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static uint8 sblParse(uint8 ch);
static uint8 sblResp(void);
static void sblExec(uint8 *pBuf);

#include "_hal_uart_isr_sbl.c"

/**************************************************************************************************
 * @fn          sblHalInit
 *
 * @brief       Serial Boot low-level initialization according to the RPC transport.
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
static void sblHalInit(void)
{
  halUARTCfg_t uartConfig;
  HalUARTInitISR();
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 0;  // CC2530 by #define - see hal_board_cfg.h
  uartConfig.rx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
  uartConfig.tx.maxBufSize        = 0;  // CC2530 by #define - see hal_board_cfg.h
  uartConfig.idleTimeout          = 0;  // CC2530 by #define - see hal_board_cfg.h
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = NULL;
  HalUARTOpenISR(&uartConfig);
}

/**************************************************************************************************
 * @fn          sblRun
 *
 * @brief       Serial Boot run code.
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
static void sblRun(void)
{
  uint8 resetF = 0;
  sblHalInit();

  while (1)
  {
    if (txLen == 0)
    {
      if ((resetF != 0) && ((UxCSR & CSR_ACTIVE) == 0))  // If the last Tx byte has flushed out.
      {
        break;
      }
    }
    else if (UTXxIF != 0)
    {
      if (--txLen != 0)
      {
        UTXxIF = 0;
      }

      UxDBUF = sbBuf[txIdx++];
    }

    if (URXxIF)
    {
      uint8 ch = UxDBUF;
      URXxIF = 0;
      resetF |= sblParse(ch);
    }
  }
}

/**************************************************************************************************
 * @fn          sblParse
 *
 * @brief       Serial Boot parser according to the RPC UART transport.
 *
 * input parameters
 *
 * @param       ch - The Rx character to parse.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
static uint8 sblParse(uint8 ch)
{
  sbBuf[sbSte + sbIdx] = ch;

  switch (sbSte)
  {
  case SBL_SOF_STATE:
    if (RPC_UART_SOF == ch)
    {
      sbSte = SBL_LEN_STATE;
    }
    break;

  case SBL_LEN_STATE:
    sbFcs = sbIdx = 0;
    sbSte = SBL_CMD1_STATE;
    sbSte = ((sbLen = ch) >= SBL_MAX_SIZE) ? SBL_SOF_STATE : SBL_CMD1_STATE;
    break;

  case SBL_CMD1_STATE:
    sbSte = SBL_CMD2_STATE;
    break;

  case SBL_CMD2_STATE:
    sbSte = (sbLen == 0) ? SBL_FCS_STATE : SBL_DATA_STATE;
    break;

  case SBL_DATA_STATE:
    if (++sbIdx == sbLen)
    {
      sbSte = SBL_FCS_STATE;
    }
    break;

  case SBL_FCS_STATE:
    sbSte = SBL_SOF_STATE;

    if ((sbFcs == ch) && ((sbBuf[SBL_CMD1_STATE] & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT))
    {
      sblExec(sbBuf + SBL_LEN_STATE);
      return sblResp();  // Send the SB response setup in the sbBuf passed to sblExec().
    }
    break;

  default:
    break;
  }

  sbFcs ^= ch;
  return FALSE;
}

/**************************************************************************************************
 * @fn          sblResp
 *
 * @brief       Make the SB response.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
static uint8 sblResp(void)
{
  uint8 idx, fcs = 0, len = sbBuf[SBL_LEN_STATE] + SBL_FCS_STATE - 1;

  for (idx = SBL_LEN_STATE; idx < len; idx++)
  {
    fcs ^= sbBuf[idx];
  }
  sbBuf[idx++] = fcs;

  txLen = idx;
  txIdx = 0;

  if ((sbBuf[SBL_CMD2_STATE] == (SBL_ENABLE_CMD | SBL_RSP_MASK)) &&
      (sbBuf[SBL_DATA_STATE] == SBL_SUCCESS))
  {
    txLen++;  // Send an extra garbage byte to flush the last good one before resetting.
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

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
 **************************************************************************************************
 */
static void sblExec(uint8 *pBuf)
{
  uint16 t16 = BUILD_UINT16(pBuf[SBL_REQ_ADDR_LSB],pBuf[SBL_REQ_ADDR_MSB]) + OAD_DONGLE_SBL_IMG_BEG;
  uint8 len = 1;
  uint8 rsp = SBL_SUCCESS;

  switch (pBuf[RPC_POS_CMD1])
  {
  case SBL_WRITE_CMD:
    if ((t16 >= OAD_DONGLE_SBL_IMG_END) || (t16 < OAD_DONGLE_SBL_IMG_BEG))
    {
      rsp = SBL_FAILURE;
      break;
    }
    if ((t16 % SBL_PAGE_SIZE) == 0)
    {
      HalFlashErase(t16 / SBL_PAGE_SIZE);
    }
    HalFlashWrite(t16, (pBuf + SBL_REQ_DAT0), (SBL_RW_BUF_LEN / HAL_FLASH_WORD_SIZE));
    break;

  case SBL_READ_CMD:
    len = SBL_RW_BUF_LEN + SBL_READ_HDR_LEN;
    pBuf[SBL_RSP_ADDR_MSB] = pBuf[SBL_REQ_ADDR_MSB];
    pBuf[SBL_RSP_ADDR_LSB] = pBuf[SBL_REQ_ADDR_LSB];

    HalFlashRead(t16 / SBL_PAGE_SIZE,
                (t16 % SBL_PAGE_SIZE) << 2, (pBuf + SBL_RSP_DAT0), SBL_RW_BUF_LEN);
    break;

  case SBL_ENABLE_CMD:
    // Bootload master must verify download by read back - no room for CRC checking code in dongle.
    break;

  case SBL_HANDSHAKE_CMD:
    break;

  default:
    rsp = SBL_FAILURE;
    break;
  }

  pBuf[RPC_POS_LEN] = len;
  pBuf[RPC_POS_CMD1] |= SBL_RSP_MASK;
  pBuf[RPC_POS_DAT0] = rsp;
}

/**************************************************************************************************
*/
