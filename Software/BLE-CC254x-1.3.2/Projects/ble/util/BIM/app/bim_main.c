/**************************************************************************************************
  Filename:       bim_main.c
  Revised:        $Date: 2012-12-05 09:57:44 -0800 (Wed, 05 Dec 2012) $
  Revision:       $Revision: 32445 $

  Description:

  This module contains the definitions for the main functionality of an Image Boot Manager.


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

#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define BIM_IMG_A_PAGE        1
#define BIM_IMG_A_AREA        62

#define BIM_IMG_B_PAGE        8
#define BIM_IMG_B_AREA       (124 - BIM_IMG_A_AREA)

#define BIM_CRC_OSET          0x00
#define BIM_HDR_OSET          0x00

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct {
  // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '<' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite().

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */

__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];

__no_init __data uint8 JumpToImageAorB @ 0x09;

#pragma location = "ALIGNED_CODE"
void halSleepExec(void);

/**************************************************************************************************
 * @fn          halSleepExec
 *
 * @brief       This function puts the CC254x to sleep by writing to the PCON register.
 *              The instruction after writing to PCON must not be 4-byte aligned or excessive
 *              power consumption may result. Since the write to PCON is 3 instructions, this
 *              function is forced to be even-byte aligned. Thus, this function must not have any
 *              automatic variables and the write to PCON must be the first C statement.
 *              See the linker file ".xcl" for actual placement of this function.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
#pragma optimize=none
void halSleepExec(void)
{
  PCON = 0x01;
  ASM_NOP;
}

/**************************************************************************************************
 * @fn          crcCalc
 *
 * @brief       Run the CRC16 Polynomial calculation over the image specified.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalc(uint8 page)
{
  HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

  const img_hdr_t *pImgHdr = (const img_hdr_t *)(pgBuf + BIM_HDR_OSET);

  uint8 pageBeg = page;
  uint8 pageEnd = pImgHdr->len / (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE);
  uint16 osetEnd = (pImgHdr->len - (pageEnd * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)))
                                                                   * HAL_FLASH_WORD_SIZE;
  pageEnd += pageBeg;
  if (pageBeg == BIM_IMG_A_PAGE)
  {
    pageEnd += BIM_IMG_B_AREA;
  }

  ADCCON1 &= 0xF3;  // CRC configuration of LRSR.

  // CRC seed of 0x0000.
  RNDL = 0x00;
  RNDL = 0x00;

  while(1)
  {
    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset++)
    {
      if ((page == pageBeg) && (oset == BIM_CRC_OSET))
      {
        oset += 3;  // Skip the CRC and shadow.
      }
      else if ((page == pageEnd) && (oset == osetEnd))
      {
        uint16 crc = RNDH;
        crc = (crc << 8) | RNDL;

        return crc;
      }
      else
      {
        RNDH = pgBuf[oset];
      }
    }

    if (++page == BIM_IMG_B_PAGE)
    {
      page += BIM_IMG_B_AREA;
    }
    HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);
  }
}

/**************************************************************************************************
 * @fn          crcCheck
 *
 * @brief       Calculate the image CRC and set it ready-to-run if it is good.
 *
 * input parameters
 *
 * @param       page - Flash page on which to beging the CRC calculation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None, but no return from this function if the CRC check is good.
 **************************************************************************************************
 */
static void crcCheck(uint8 page, uint16 *crc)
{
  HAL_BOARD_INIT();

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

  if (crc[0] == crcCalc(page))
  {
    uint16 addr = page * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE) +
                                 BIM_CRC_OSET / HAL_FLASH_WORD_SIZE;
    crc[1] = crc[0];
    crc[0] = 0xFFFF;

    HalFlashWrite(addr, (uint8 *)crc, 1);
    HAL_SYSTEM_RESET();
  }
}

/**************************************************************************************************
 * @fn          main
 *
 * @brief       C-code main function.
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
 **************************************************************************************************
 */
void main(void)
{
  uint16 crc[2];

  // Prefer to run Image-B over Image-A so that Image-A does not have to invalidate itself.
  HalFlashRead(BIM_IMG_B_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 1;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x4030");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    /* This check is disruptive when an OAD process to Image-A is interrupted - this check must
     * complete before the still good Image-A is run.
    else if (crc[1] == 0xFFFF)  // If first run of an image that was physically downloaded.
    {
      crcCheck(BIM_IMG_B_PAGE, crc);
    }*/
  }

  HalFlashRead(BIM_IMG_A_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 0;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x0830");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    else if (crc[1] == 0xFFFF)  // If first run of an image that was physically downloaded.
    {
      crcCheck(BIM_IMG_A_PAGE, crc);
    }
  }

  SLEEPCMD |= 0x03;  // PM3, All clock oscillators off, voltage regulator off.
  halSleepExec();
  HAL_SYSTEM_RESET();  // Should not get here.
}

/**************************************************************************************************
*/
