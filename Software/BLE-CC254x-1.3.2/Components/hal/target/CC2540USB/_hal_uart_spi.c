/**************************************************************************************************
  Filename:       _hal_uart_spi.c
  Revised:        $Date: 2013-02-06 09:21:21 -0800 (Wed, 06 Feb 2013) $
  Revision:       $Revision: 33001 $

  Description:    Implementation of a specialized SPI Transport driver for emulating a UART API.

  Concept of Operation for porting this code to act as a SPI Master:

  5 GPIO lines are required: MOSI, MISO, SPI Clock, SPI CSn/MRDY, and SRDY.
    MRDY = SPI Master Ready to send and serves as a SPI Slave wakeup from sleep.
    SRDY = SPI Slave Ready to send and serves as a SPI Master wakeup from sleep.

  SRDY also serves as an indication that the Slave has awoken from sleep and is ready to recieve
  as follows: when the Slave has no message ready, it will toggle the state of its SRDY on every
  falling edge of the CSn/MRDY. The SPI Slave has a configurable time delay before re-enabling
  sleep after being awoken.

  The SPI Slave does not re-enter sleep for the following conditions:
  - Slave is asserting SRDY (i.e. it is waiting to send a SPI packet).
  - Master is asserting MRDY.
  - Slave has parsed the SOF byte of a SPI packet, but has not received the entire packet yet.
    Note how important this case is to support a master that has to interrupt a SPI packet
    transmission in order to service a higher priority SPI slave (i.e. the master has executed the
    protocol to ensure that the SPI slave is awake, but cannot hold MRDY low for the entire,
    contiguous packet for having to service some other SPI slave mid-packet).

  Any transfer, from one byte to SPI_MAX_DAT_LEN bytes is packetized by pre-pending an SOF & LEN
  byte and appending an FCS byte. The FCS is calculated over the data payload and the LEN bytes.

  Only Full-Duplex transfers are supported - at any byte of a Tx transfer,
  the incoming bytes may not be just garbage bytes clocked out by the synchronous transfer, but
  instead be meaningful bytes of a Tx-packet from the other side. Thus, the necessity of concocting
  the concept of the SPI transfer packet described above.

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

#include <string.h>

#include "hal_assert.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_drivers.h"
#include "hal_dma.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_uart.h"
#if defined POWER_SAVING
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#elif defined HAL_SPI_QUEUED_TX && HAL_SPI_QUEUED_TX
#include "OSAL.h"
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* The bridge utility application uses 4 DMA channels for 2 sets of Rx/Tx by DMA, and thereby also
 * implements a special DMA ISR handler, which accomodates this extra definition.
 */
#if !defined HAL_SPI_CH_RX
#define HAL_SPI_CH_RX              HAL_DMA_CH_RX
#endif
#if !defined HAL_SPI_CH_TX
#define HAL_SPI_CH_TX              HAL_DMA_CH_TX
#endif

// Overlay the SPI Rx/Tx buffers on the UART Rx/Tx buffers when running with a run-time selectable
// UART port by SPI or by UART in order to save XDATA space.
#if !defined HAL_SPI_ON_UART_BUFS
#define HAL_SPI_ON_UART_BUFS       FALSE
#endif

#if !defined HAL_SPI_QUEUED_TX
#define HAL_SPI_QUEUED_TX          FALSE
#endif

#define SPI_MAX_PKT_LEN            256
#define SPI_MAX_DAT_LEN           (SPI_MAX_PKT_LEN - SPI_FRM_LEN)

#define SPI_SOF                    0xFE  // Start-of-frame delimiter for SPI transport.

// The FCS is calculated over the SPI Frame Header and the Frame Data bytes.
#define SPI_HDR_LEN                1     // One byte LEN pre-pended to data byte array.
// SOF & Header bytes pre-pended and the FCS byte appended.
#define SPI_FRM_LEN               (2 + SPI_HDR_LEN)

#define SPI_SOF_IDX                0
#define SPI_HDR_IDX                1     // The frame header consists only of the LEN byte for now.
#define SPI_LEN_IDX                1     // LEN byte is offset by the SOF byte.
#define SPI_DAT_IDX                2     // Data bytes are offset by the SOF & LEN bytes.

#if defined HAL_SPI_MASTER
// If POWER_SAVING is enabled in the 8051-SOC slave, then any master transmit must be preceded by a
// dummy zero byte transfer to give the slave time to wake from PM2/3 & enable its 1.8V and signal
// such success by responding with its SRDY active low.
#if !defined HAL_SPI_WAKEUP
#define HAL_SPI_WAKEUP             TRUE
#endif
#endif

#if !defined HAL_UART_SPI
#define HAL_UART_SPI               2
#elif  (HAL_UART_SPI != 2)
#error Only supporting HAL_UART_SPI==2 for now.
#endif

// UxCSR - USART Control and Status Register.
#define CSR_MODE                   0x80
#define CSR_RE                     0x40
#define CSR_SLAVE                  0x20
#define CSR_FE                     0x10
#define CSR_ERR                    0x08
#define CSR_RX_BYTE                0x04
#define CSR_TX_BYTE                0x02
#define CSR_ACTIVE                 0x01

// UxUCR - USART UART Control Register.
#define UCR_FLUSH                  0x80
#define UCR_FLOW                   0x40
#define UCR_D9                     0x20
#define UCR_BIT9                   0x10
#define UCR_PARITY                 0x08
#define UCR_SPB                    0x04
#define UCR_STOP                   0x02
#define UCR_START                  0x01

#define P2DIR_PRIPO                0xC0

#define HAL_DMA_U0DBUF             0x70C1
#define HAL_DMA_U1DBUF             0x70F9

#undef  UxCSR
#undef  UxUCR
#undef  UxDBUF
#undef  UxBAUD
#undef  UxGCR
#if    (HAL_UART_SPI == 1)
#define UxCSR                      U0CSR
#define UxUCR                      U0UCR
#define UxDBUF                     U0DBUF
#define UxBAUD                     U0BAUD
#define UxGCR                      U0GCR
#elif  (HAL_UART_SPI == 2)
#define UxCSR                      U1CSR
#define UxUCR                      U1UCR
#define UxDBUF                     U1DBUF
#define UxBAUD                     U1BAUD
#define UxGCR                      U1GCR
#endif

#undef  PxSEL
#undef  HAL_UART_PERCFG_BIT
#undef  HAL_UART_PRIPO
#undef  HAL_UART_Px_SEL_S
#undef  HAL_UART_Px_SEL_M
#if    (HAL_UART_SPI == 1)
#define PxSEL                      P0SEL
#define HAL_UART_PERCFG_BIT        0x01         // USART0 on P0, Alt-1; so clear this bit.
#define HAL_UART_PRIPO             0x00         // USART0 priority over UART1.
#define HAL_UART_Px_SEL_S          0x3C         // Peripheral I/O Select for Slave: SO/SI/CLK/CSn.
#define HAL_UART_Px_SEL_M          0x2C         // Peripheral I/O Select for Master: MI/MO/CLK.
#elif  (HAL_UART_SPI == 2)
#define PxSEL                      P1SEL
#define HAL_UART_PERCFG_BIT        0x02         // USART1 on P1, Alt-2; so set this bit.
#define HAL_UART_PRIPO             0x40         // USART1 priority over UART0.
#define HAL_UART_Px_SEL_S          0xF0         // Peripheral I/O Select for Slave: SO/SI/CLK/CSn.
#define HAL_UART_Px_SEL_M          0xE0         // Peripheral I/O Select for Master: MI/MO/CLK.
#endif

#undef  DMA_PAD
#undef  DMA_UxDBUF
#undef  DMATRIG_RX
#undef  DMATRIG_TX
#if    (HAL_UART_SPI == 1)
#define DMA_PAD                    U0BAUD
#define DMA_UxDBUF                 HAL_DMA_U0DBUF
#define DMATRIG_RX                 HAL_DMA_TRIG_URX0
#define DMATRIG_TX                 HAL_DMA_TRIG_UTX0
#elif  (HAL_UART_SPI == 2)
#define DMA_PAD                    U1BAUD
#define DMA_UxDBUF                 HAL_DMA_U1DBUF
#define DMATRIG_RX                 HAL_DMA_TRIG_URX1
#define DMATRIG_TX                 HAL_DMA_TRIG_UTX1
#endif

#undef  PxIEN
#undef  PxIFG
#undef  PxIF
#undef  PICTL_BIT
#undef  IENx
#undef  IEN_BIT
#if   ((HAL_UART_SPI == 1) || defined HAL_SPI_MASTER)
#define PxIEN                      P0IEN
#define PxIFG                      P0IFG
#define PxIF                       P0IF
#if !defined SPI_RDYIn
#define SPI_RDYIn                  P0_1
#define SPI_RDYIn_BIT              BV(1)
#endif
// Falling edge ISR on P0 pins.
#define PICTL_BIT                  BV(0)
#define IENx                       IEN1
#define IEN_BIT                    BV(5)
#elif  (HAL_UART_SPI == 2)
#define PxIEN                      P1IEN
#define PxIFG                      P1IFG
#define PxIF                       P1IF
#define SPI_RDYIn                  P1_4
#define SPI_RDYIn_BIT              BV(4)
// Falling edge ISR on P1.4-7 pins.
#define PICTL_BIT                  BV(2)
#define IENx                       IEN2
#define IEN_BIT                    BV(4)
#endif

#undef  PxDIR
#if   ((HAL_UART_SPI == 1) || defined HAL_SPI_MASTER)
#define PxDIR                      P1DIR
#define SPI_RDYOut                 P1_4
#define SPI_RDYOut_BIT             BV(4)
#elif  (HAL_UART_SPI == 2)
#define PxDIR                      P0DIR
#define SPI_RDYOut                 P0_5
#define SPI_RDYOut_BIT             BV(5)
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           TypeDefs
 * ------------------------------------------------------------------------------------------------
 */

#if (SPI_MAX_PKT_LEN <= 256)
typedef uint8  spiLen_t;
#else
typedef uint16 spiLen_t;
#endif

typedef enum
{
  spiRxSteSOF,
  spiRxSteLen,
  spiRxSteData,
  spiRxSteFcs
} spiRxSte_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#if SPI_MAX_PKT_LEN == 256
#define SPI_LEN_T_INCR(LEN)  (LEN)++
#define SPI_LEN_T_BOUNDS_CHECK(LEN)
#else
#define SPI_LEN_T_INCR(LEN) st (  \
  if (++(LEN) >= SPI_MAX_PKT_LEN) \
  { \
    (LEN) = 0; \
  } \
)

#define SPI_LEN_T_BOUNDS_CHECK(LEN) st ( \
  if (len > SPI_MAX_PKT_LEN) \
  { \
    len = SPI_MAX_PKT_LEN; \
  } \
)
#endif

#define SPI_CLR_RX_BYTE(IDX)   (spiRxBuf[(IDX)] = BUILD_UINT16(0, (DMA_PAD ^ 0xFF)))
#define SPI_GET_RX_BYTE(IDX)   (*(volatile uint8 *)(spiRxBuf+(IDX)))
#define SPI_NEW_RX_BYTE(IDX)   ((uint8)DMA_PAD == HI_UINT16(spiRxBuf[(IDX)]))

#define SPI_DAT_LEN(PBUF)      ((PBUF)[SPI_LEN_IDX])
#define SPI_PKT_LEN(PBUF)      (SPI_DAT_LEN((PBUF)) + SPI_FRM_LEN)

#define SPI_RDY_IN()           (SPI_RDYIn == 0)
#define SPI_RDY_OUT()          (SPI_RDYOut == 0)

#if defined HAL_SPI_MASTER
#define SPI_CLR_CSn_OUT()      (SPI_RDYOut = 1)
#define SPI_SET_CSn_OUT()      (SPI_RDYOut = 0)

#define SPI_CLOCK_RX(CNT) st ( \
  SPI_SET_CSn_OUT(); \
  \
  for (spiLen_t cnt = 0; cnt <= (CNT); cnt++) \
  { \
    UxDBUF = 0; \
    while (UxCSR & CSR_ACTIVE); \
  } \
  \
  SPI_CLR_CSn_OUT(); \
)
#define SPI_CLR_RDY_OUT()       SPI_CLR_CSn_OUT()

#elif !defined HAL_SPI_MASTER
#define SPI_CLR_RDY_OUT()      (SPI_RDYOut = 1)
#define SPI_SET_RDY_OUT()      (SPI_RDYOut = 0)
#define SPI_TOG_RDY_OUT() st ( \
  if (SPI_RDY_OUT()) \
  { \
    SPI_CLR_RDY_OUT(); \
  } \
  else \
  { \
    SPI_SET_RDY_OUT(); \
  } \
)
#endif

#define SPI_RX_RDY()            (spiRxHead != spiRxTail)

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// Convenient way to save RAM space when running with both SPI & UART ports enabled,
// but only using one at a time with a run-time choice.
#if HAL_SPI_ON_UART_BUFS
static uint16 *const spiRxBuf = dmaCfg.rxBuf;
#if (defined HAL_UART_TX_BY_ISR && HAL_UART_TX_BY_ISR)
static uint8 *const spiRxDat = dmaCfg.txBuf;
static __no_init uint8  spiTxPkt[SPI_MAX_PKT_LEN];
#else
static uint8 *const spiRxDat = dmaCfg.txBuf[0];
static uint8 *const spiTxPkt = dmaCfg.txBuf[1];
#endif
#else
static __no_init uint16 spiRxBuf[SPI_MAX_PKT_LEN];  // uint16 for trick with DMA Rx 2-for-1.
static __no_init uint8  spiRxDat[SPI_MAX_PKT_LEN];
static __no_init uint8  spiTxPkt[SPI_MAX_PKT_LEN];
#endif

#if HAL_SPI_QUEUED_TX
static osal_msg_q_t spiTxQ;
static volatile uint8 spiTxMT;
#endif

static spiRxSte_t spiRxSte;  // State of SPI packet parsing from spiRxBuf[].
static spiLen_t spiRxIdx;    // Index in spiRxBuf[] for SPI packet parsing.
static spiLen_t spiRxCnt;    // Count payload data bytes parsed from spiRxBuf[].
static spiLen_t spiRxLen;    // Total length of payload data bytes to be parsed from spiRxBuf[].
static spiLen_t spiRxFcs;    // Running FCS calculation of bytes being parsed from spiRxBuf[].

static spiLen_t spiRxHead;   // Index in spiRxDat[] from which to read with HalUARTReadSPI().
static spiLen_t spiRxTail;   // Index in spiRxDat[] up to which to read with HalUARTReadSPI().
static spiLen_t spiRxTemp;   // Index in spiRxDat[] to which to parse data from spiRxBuf[].

// Not supporting Tx packet buffering with a spiTxBuf[] for now to save XDATA/Flash space.

static volatile spiLen_t spiTxLen;  // Total length of payload data bytes to transmit.

static halUARTCBack_t spiCB;

#if (defined POWER_SAVING || (defined HAL_SPI_MASTER && HAL_SPI_WAKEUP))
static volatile uint8 spiRdyIsr;  // Sticky flag from SPI_RDY_IN ISR for background polling.
#endif

#if defined POWER_SAVING
static uint8 spiHoldPM;    // Boolean whether or not this module is holding off the PM.
#endif

/* Used to walk the spiRxBuf[] one byte per polling pass as a work-around for the case when
 * full-duplex traffic using Rx & Tx DMA running simultaneously. The immediate buffer area around
 * the spiRxIdx consists of "cleared" uint16 values, but 10-16 indices ahead, there are valid SPI
 * packets ready to be parsed.
 */
static spiLen_t spiRxBug;  // DMA pointer is seen to jump ahead, or S/W bug corrupts spiRxIdx?

/* ------------------------------------------------------------------------------------------------
 *                                          Global Functions
 * ------------------------------------------------------------------------------------------------
 */

void HalUART_DMAIsrSPI(void);

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static uint8 spiCalcFcs(uint8 *pBuf);
static void spiParseRx(void);
#if defined POWER_SAVING
static uint8 spiSweepRx(void);
#endif

/**************************************************************************************************
 * @fn          HalUARTInitSPI
 *
 * @brief       Initialize the SPI UART Transport.
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
static void HalUARTInitSPI(void)
{
#if (HAL_UART_SPI == 1)
  PERCFG &= ~HAL_UART_PERCFG_BIT;    // Set UART0 I/O to Alt. 1 location on P0.
#else
  PERCFG |= HAL_UART_PERCFG_BIT;     // Set UART1 I/O to Alt. 2 location on P1.
#endif
#if defined HAL_SPI_MASTER
  PxSEL |= HAL_UART_Px_SEL_M;        // SPI-Master peripheral select.
  UxCSR = 0;                         // Mode is SPI-Master Mode.
  UxGCR = 16;                        // Cfg for the max Rx/Tx baud of 4-MHz.
  UxBAUD = 255;
#elif !defined HAL_SPI_MASTER
  PxSEL |= HAL_UART_Px_SEL_S;        // SPI-Slave peripheral select.
  UxCSR = CSR_SLAVE;                 // Mode is SPI-Slave Mode.
#endif
  UxUCR = UCR_FLUSH;                 // Flush it.
  UxGCR |= BV(5);                    // Set bit order to MSB.

  P2DIR &= ~P2DIR_PRIPO;
  P2DIR |= HAL_UART_PRIPO;

  // Setup GPIO for interrupts by falling edge on SPI_RDY_IN.
  PxIEN |= SPI_RDYIn_BIT;
  PICTL |= PICTL_BIT;

  SPI_CLR_RDY_OUT();
  PxDIR |= SPI_RDYOut_BIT;

  // Setup Tx by DMA.
  halDMADesc_t *ch = HAL_DMA_GET_DESC1234( HAL_SPI_CH_TX );

  // The start address of the destination.
  HAL_DMA_SET_DEST( ch, DMA_UxDBUF );

  // Using the length field to determine how many bytes to transfer.
  HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

  // One byte is transferred each time.
  HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_BYTE );

  // The bytes are transferred 1-by-1 on Tx Complete trigger.
  HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE );
  HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_TX );

  // The source address is incremented by 1 byte after each transfer.
  HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_1 );
  HAL_DMA_SET_SOURCE( ch, spiTxPkt );

  // The destination address is constant - the Tx Data Buffer.
  HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_0 );

  // The DMA Tx done is serviced by ISR in order to maintain full thruput.
  HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_ENABLE );

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH );

  // Setup Rx by DMA.
  ch = HAL_DMA_GET_DESC1234( HAL_SPI_CH_RX );

  // The start address of the source.
  HAL_DMA_SET_SOURCE( ch, DMA_UxDBUF );

  // Using the length field to determine how many bytes to transfer.
  HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

  /* The trick is to cfg DMA to xfer 2 bytes for every 1 byte of Rx.
   * The byte after the Rx Data Buffer is the Baud Cfg Register,
   * which always has a known value. So init Rx buffer to inverse of that
   * known value. DMA word xfer will flip the bytes, so every valid Rx byte
   * in the Rx buffer will be preceded by a DMA_PAD char equal to the
   * Baud Cfg Register value.
   */
  HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_WORD );

  // The bytes are transferred 1-by-1 on Rx Complete trigger.
  HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE_REPEATED );
  HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_RX );

  // The source address is constant - the Rx Data Buffer.
  HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_0 );

  // The destination address is incremented by 1 word after each transfer.
  HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_1 );
  HAL_DMA_SET_DEST( ch, spiRxBuf );
  HAL_DMA_SET_LEN( ch, SPI_MAX_PKT_LEN );

  // The DMA is to be polled and shall not issue an IRQ upon completion.
  HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_DISABLE );

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH );

  volatile uint8 dummy = *(volatile uint8 *)DMA_UxDBUF;  // Clear the DMA Rx trigger.
  HAL_DMA_CLEAR_IRQ(HAL_SPI_CH_RX);
  HAL_DMA_ARM_CH(HAL_SPI_CH_RX);
  (void)memset(spiRxBuf, (DMA_PAD ^ 0xFF), SPI_MAX_PKT_LEN * sizeof(uint16));
}

/**************************************************************************************************
 * @fn          HalUARTOpenSPI
 *
 * @brief       Start the SPI UART Transport.
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
static void HalUARTOpenSPI(halUARTCfg_t *config)
{
  spiCB = config->callBackFunc;

  UxCSR |= CSR_RE;

  PxIFG = 0;
  PxIF = 0;
  IENx |= IEN_BIT;
}

/**************************************************************************************************
 * @fn          HalUARTReadSPI
 *
 * @brief       Read data bytes from a received SPI packet.
 *
 * input parameters
 *
 * @param       buf - pointer to the memory to where to copy the received SPI data bytes.
 * @param       len - the length to copy (for now, must be the exact length of the received data).
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of bytes read from a received SPI packet.
 */
static spiLen_t HalUARTReadSPI(uint8 *buf, spiLen_t len)
{
  for (spiLen_t cnt = 0; cnt < len; cnt++)
  {
    if (spiRxHead == spiRxTail)
    {
      len = cnt;
      break;
    }

    *buf++ = spiRxDat[spiRxHead];
    SPI_LEN_T_INCR(spiRxHead);
  }

  return len;
}

/**************************************************************************************************
 * @fn          HalUARTWriteSPI
 *
 * @brief       Transmit data bytes as a SPI packet.
 *
 * input parameters
 *
 * @param       buf - pointer to the memory of the data bytes to send.
 * @param       len - the length of the data bytes to send.
 *
 * output parameters
 *
 * None.
 *
 * @return      Zero for any error; otherwise, 'len'.
 */
static spiLen_t HalUARTWriteSPI(uint8 *buf, spiLen_t len)
{
  if (spiTxLen != 0)
  {
#if HAL_SPI_QUEUED_TX
    uint8 *txMsg = osal_msg_allocate(len);

    if (txMsg != NULL)
    {
      (void)memcpy(txMsg, buf, len);
      osal_msg_enqueue(&spiTxQ, txMsg);

      return len;
    }
#endif

    return 0;
  }

  if (len > SPI_MAX_DAT_LEN)
  {
    len = SPI_MAX_DAT_LEN;
  }
  spiTxLen = len;
#if HAL_SPI_QUEUED_TX
  spiTxMT = FALSE;
#endif

  spiTxPkt[SPI_LEN_IDX] = len;
  (void)memcpy(spiTxPkt + SPI_DAT_IDX, buf, len);

  spiCalcFcs(spiTxPkt);
  spiTxPkt[SPI_SOF_IDX] = SPI_SOF;

  halDMADesc_t *ch = HAL_DMA_GET_DESC1234(HAL_SPI_CH_TX);
  HAL_DMA_SET_LEN(ch, SPI_PKT_LEN(spiTxPkt));

#if defined HAL_SPI_MASTER
#if HAL_SPI_WAKEUP
  /* A Full-Duplex Slave will keep SRDY active low if it has Tx ready; otherwise it will toggle its
   * SRDY with every falling edge of MRDY. So if SRDY is high and the wait after writing each byte
   * is not long enough, the master may loop here, infinitely out of sync with the toggling SRDY.
   * Thus in a threaded OS, it would be advisable to send one zero and yield for 1-msec or more, or
   * be able to receive and check here for the SRDY ISR.
   */
  spiRdyIsr = 0;
  while (!SPI_RDY_IN() && (spiRdyIsr == 0))
  {
    SPI_CLOCK_RX(1);

    for (uint16 cnt = 0; cnt < 512; cnt++)
    {
      if (SPI_RDY_IN() || (spiRdyIsr != 0))
      {
        break;
      }
    }
  }
#endif
  SPI_SET_CSn_OUT();
  HAL_DMA_ARM_CH(HAL_SPI_CH_TX);
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  HAL_DMA_MAN_TRIGGER(HAL_SPI_CH_TX);

#elif !defined HAL_SPI_MASTER

  HAL_DMA_ARM_CH(HAL_SPI_CH_TX);
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  SPI_SET_RDY_OUT();
#endif

  return len;
}

/**************************************************************************************************
 * @fn          HalUARTPollSPI
 *
 * @brief       SPI Transport Polling Manager.
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
static void HalUARTPollSPI(void)
{
#if HAL_SPI_QUEUED_TX
  /* There should not be a race with the Tx-Done ISR checking the spiTxQ because if the background
   * can first detect the volatile variable spiTxLen as zero and then detect the spiTxQ as not
   * empty, then the Tx-Done ISR has already fired after the background write started to enqueue
   * the message but before it was done doing so.
   */
  if ((spiTxLen == 0) && (spiTxQ != NULL))
  {
    uint8 *txMsg = osal_msg_dequeue(&spiTxQ);

    (void)HalUARTWriteSPI(txMsg, OSAL_MSG_LEN(txMsg));
    (void)osal_msg_deallocate(txMsg);
  }
  else if (spiTxMT)
  {
    spiTxMT = FALSE;
    spiCB((HAL_UART_SPI - 1), HAL_UART_TX_EMPTY);
  }
#endif

  spiParseRx();

#if HAL_SPI_QUEUED_TX || defined HAL_SPI_MASTER
  if (SPI_RX_RDY())
#else
  // When there is no ability to queue Tx data and there is still Tx data pending, do not call the
  // Application with new Rx data, as this may result in an attempt to respond. Although this
  // failure will be actionable by the Application when HalUARTWriteSPI() returns zero, simple
  // applications (like the SBL, which does not enable Tx queueing) do not handle the zero return.
  if (SPI_RX_RDY() && !spiTxLen)
#endif
  {
    if (spiCB != NULL)
    {
      spiCB((HAL_UART_SPI - 1), HAL_UART_RX_TIMEOUT);
    }
  }

  PxIEN &= ~SPI_RDYIn_BIT;  // Clear to not race with SPI_RDY_IN ISR.

  if (!SPI_RDY_IN() && !spiTxLen)
  {
    SPI_CLR_RDY_OUT();  // Clear the ready-out bit before re-enabling the ready-in ISR.
  }

#if defined POWER_SAVING
  if (SPI_RDY_IN() || SPI_RX_RDY() || spiRxLen || spiTxLen || spiRdyIsr)
  {
    spiRdyIsr = 0;  // Clear the ISR-trap flag before re-enabling the corresponding ISR.

    if (spiHoldPM == 0)
    {
      spiHoldPM = 1;
      (void)osal_set_event(Hal_TaskID, HAL_PWRMGR_HOLD_EVENT);
    }
  }
  else
  {
    spiRdyIsr = 0;  // Clear the ISR-trap flag before re-enabling the corresponding ISR.

    if (spiHoldPM != 0)
    {
      PxIEN |= SPI_RDYIn_BIT;  // Allow the ready-in ISR during the relatively long Rx sweep.

      if (spiSweepRx())
      {
        spiHoldPM = 0;
        (void)osal_set_event(Hal_TaskID, HAL_PWRMGR_CONSERVE_EVENT);
      }
    }
  }
#endif

  PxIEN |= SPI_RDYIn_BIT;
}

/**************************************************************************************************
 * @fn          HalUARTRxAvailSPI
 *
 * @brief       Return the byte count of Rx data ready to be read HalUARTReadSPI().
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      Number of bytes ready to be read with HalUARTReadSPI().
 */
static spiLen_t HalUARTRxAvailSPI(void)
{
  return (spiRxTail > spiRxHead) ? (spiRxTail-spiRxHead) : (spiRxTail+(SPI_MAX_PKT_LEN-spiRxHead));
}

/**************************************************************************************************
 * @fn          spiCalcFcs
 *
 * @brief       Calculate the FCS of a SPI Transport frame and append it to the packet.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the SPI Transport frame over which to calculate the FCS.
 *
 * output parameters
 *
 * None.
 *
 * @return      The FCS calculated.
 */
static uint8 spiCalcFcs(uint8 *pBuf)
{
  spiLen_t len = SPI_DAT_LEN(pBuf) + SPI_HDR_LEN + SPI_HDR_IDX;
  spiLen_t idx = SPI_HDR_IDX;
  uint8 fcs = 0;

  while (idx < len)
  {
    fcs ^= pBuf[idx++];
  }
  pBuf[idx] = fcs;

  return fcs;
}

/**************************************************************************************************
 * @fn          spiParseRx
 *
 * @brief       Parse all available bytes from the spiRxBuf[]; parse Rx data into the spiRxDat[].
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
static void spiParseRx(void)
{
  while (1)
  {
    if (!SPI_NEW_RX_BYTE(spiRxIdx))
    {
#if defined HAL_SPI_MASTER
      if (SPI_RDY_IN() && (spiTxLen == 0))
      {
        SPI_CLOCK_RX(1);
        continue;
      }
#endif

      if (SPI_NEW_RX_BYTE(spiRxBug))
      {
        while (!SPI_NEW_RX_BYTE(spiRxIdx))
        {
          SPI_LEN_T_INCR(spiRxIdx);
        }

        spiRxBug = spiRxIdx;
        continue;
      }

      SPI_LEN_T_INCR(spiRxBug);
      break;
    }

    uint8 ch = SPI_GET_RX_BYTE(spiRxIdx);
    SPI_CLR_RX_BYTE(spiRxIdx);
    SPI_LEN_T_INCR(spiRxIdx);

    switch (spiRxSte)
    {
    case spiRxSteSOF:
      if (ch == SPI_SOF)
      {
        spiRxSte = spiRxSteLen;

        // At this point, the master has effected the protocol for ensuring that the SPI slave is
        // awake, so set the spiRxLen to non-zero to prevent the slave from re-entering sleep until
        // the entire packet is received - even if the master interrupts the sending of the packet
        // by de-asserting/re-asserting MRDY one or more times.
        spiRxLen = 1;
      }
      break;

    case spiRxSteLen:
      if ((ch == 0) || (ch > SPI_MAX_DAT_LEN))
      {
        spiRxSte = spiRxSteSOF;
        spiRxLen = 0;
      }
      else
      {
        spiRxFcs = spiRxLen = ch;
        spiRxTemp = spiRxTail;
        spiRxCnt = 0;
        spiRxSte = spiRxSteData;
#if defined HAL_SPI_MASTER
        SPI_CLOCK_RX(ch + 1);  // Clock out the SPI Frame Data bytes and FCS.
#endif
      }
      break;

    case spiRxSteData:
      spiRxFcs ^= ch;
      spiRxDat[spiRxTemp] = ch;
      SPI_LEN_T_INCR(spiRxTemp);

      if (++spiRxCnt == spiRxLen)
      {
        spiRxSte = spiRxSteFcs;
      }
      break;

    case spiRxSteFcs:
      spiRxSte = spiRxSteSOF;

      if (ch == spiRxFcs)
      {
        spiRxTail = spiRxTemp;
      }

      spiRxCnt = spiRxLen = 0;
      break;

    default:
      HAL_ASSERT(0);
      break;
    }
  }
}

#if defined POWER_SAVING
/**************************************************************************************************
 * @fn          spiSweepRx
 *
 * @brief       Sweep the entire spiRxBuf[] looking for buggy DMA Rx - this function is an
 *              enhancement to the 'spiRxBug' work-around that is required when POWER_SAVING is
 *              enabled because the 'spiRxBug' only looks at one byte per polling pass and the
 *              buggy DMA Rx may be further away and not encountered before allowing sleep.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * The local global variables spiRxIdx and spiRxBug are set to the index of the first valid Rx
 * after the index where spiRxIdx had been set upon entering this function.
 *
 * @return      TRUE for a clean sweep (spiRxBuf[] has no valid Rx); FALSE otherwise.
 */
static uint8 spiSweepRx(void)
{
  spiLen_t sweepIdx = spiRxIdx;

  do {
    if (SPI_NEW_RX_BYTE(sweepIdx))
    {
      spiRxBug = spiRxIdx = sweepIdx++;
      break;
    }

    SPI_LEN_T_INCR(sweepIdx);

  } while (sweepIdx != spiRxIdx);

  return (sweepIdx == spiRxIdx);
}
#endif

/**************************************************************************************************
 * @fn          HalUART_DMAIsrSPI
 *
 * @brief       Handle the Tx done DMA ISR.
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
void HalUART_DMAIsrSPI(void)
{
#if !defined HAL_SPI_MASTER
  if (!SPI_RDY_IN())
#endif
  {
    SPI_CLR_RDY_OUT();
  }
  spiTxLen = 0;

#if HAL_SPI_QUEUED_TX
  if (spiTxQ != NULL)
  {
    uint8 *txMsg = osal_msg_dequeue(&spiTxQ);

    (void)HalUARTWriteSPI(txMsg, OSAL_MSG_LEN(txMsg));
    (void)osal_msg_deallocate(txMsg);
  }
  else
  {
    spiTxMT = TRUE;
  }
#else
  spiCB((HAL_UART_SPI - 1), HAL_UART_TX_EMPTY);
#endif
}

/**************************************************************************************************
 * @fn      PortX Interrupt Handler
 *
 * @brief   Handler for the SPI_RDY_IN falling-edge ISR.
 *
 * @param   None.
 *
 * @return  None.
 *************************************************************************************************/
#if defined HAL_SBL_BOOT_CODE
static void spiRdyIn(void)
{
#else
#if  ((HAL_UART_SPI == 1) || defined HAL_SPI_MASTER)
HAL_ISR_FUNCTION(port0Isr, P0INT_VECTOR)
#elif (HAL_UART_SPI == 2)
HAL_ISR_FUNCTION(port1Isr, P1INT_VECTOR)
#endif
{
  HAL_ENTER_ISR();
#endif

  PxIFG = 0;
  PxIF = 0;

#if (defined POWER_SAVING || (defined HAL_SPI_MASTER && HAL_SPI_WAKEUP))
  spiRdyIsr = 1;
#endif
#if !defined HAL_SPI_MASTER
  if (spiTxLen == 0)
  {
    SPI_TOG_RDY_OUT();
  }
#endif

#if !defined HAL_SBL_BOOT_CODE
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
#endif
}

/**************************************************************************************************
*/
