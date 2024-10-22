/**************************************************************************************************
  Filename:       cma3000d.c
  Revised:        $Date: 2013-02-08 05:16:52 -0800 (Fri, 08 Feb 2013) $
  Revision:       $Revision: 33023 $

  Description:    Control of the accelerometer on the keyfob board in the CC2540DK-mini
                  kit.

  Copyright 2009 - 2010 Texas Instruments Incorporated. All rights reserved.

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

#include <ioCC2540.h>
#include "cma3000d.h"


//***********************************************************************************
// Defines

// Accelerometer connected at (rev0.6/rev1.0):
// P0_6 = DATA_READY/-
// P1_2 = -/CS
// P1_3 = SCK
// P1_4 = MISO
// P1_5 = MOSI
// P1_7 = CS/DATA_READY

#define SCK             P1_3
#define MISO            P1_4
#define MOSI            P1_5

#ifdef REV_1_0
  #define CS              P1_2
#elif (defined REV_0_6)
  #define CS              P1_7
#endif

#define CS_DISABLED     1
#define CS_ENABLED      0


//***********************************************************************************
// Function prototypes
void spiWriteByte(uint8 write);
void spiReadByte(uint8 *read, uint8 write);


//***********************************************************************************
// Local variables
static uint8 acc_initialized = FALSE;



/** \brief	Initialize SPI interface and CMA3000-D01 accelerometer
*
* This will initialize the SPI interface and CMA3000-D01 accelerometer
*
*/
void accInit(void)
{
    //*** Setup USART 0 SPI at alternate location 2 ***

    // USART 0 at alternate location 2
    PERCFG |= 0x01;
    // Peripheral function on SCK, MISO and MOSI (P1_3-5)
    P1SEL |= 0x38;
    // Configure CS (P1_7/P1_2) as output
#ifdef REV_1_0
    P1DIR |= 0x04;
#elif (defined REV_0_6)
    P1DIR |= 0x80;
#endif
    CS = CS_DISABLED;

    //*** Setup the SPI interface ***
    // SPI master mode
    U0CSR = 0x00;
    // Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
    //                                 data in on CPOL-inv -> CPOL
    // MSB first
    U0GCR = 0x20;
    // SCK frequency = 480.5kHz (max 500kHz)
    U0GCR |= 0x0D;
    U0BAUD = 0xEC;

    uint8 readValue;
    accWriteReg(CTRL, RANGE_2G | MODE_100HZ_MEAS);
    WAIT_1_3US(80);
    do{
        accReadReg(STATUS, &readValue);
        WAIT_1_3US(80);
    }while(readValue & 0x08);
    acc_initialized = TRUE;
}

/** \brief	Sets the CMA3000-D01 accelerometer in Power Down mode
*
*/
void accStop(void)
{
  if (acc_initialized) {
     accWriteReg(CTRL, MODE_PD);
     acc_initialized = FALSE;
  }
}

/** \brief	Write one byte to a sensor register
*
* Write one byte to a sensor register
*
* \param[in]       reg
*     Register address
* \param[in]       val
*     Value to write
*/
void accWriteReg(uint8 reg, uint8 val)
{
    CS = CS_ENABLED;
    spiWriteByte(reg|0x02);
    spiWriteByte(val);
    CS = CS_DISABLED;
}


/** \brief	Read one byte from a sensor register
*
* Read one byte from a sensor register
*
* \param[in]       reg
*     Register address
* \param[in]       *pVal
*     Pointer to variable to put read out value
*/
void accReadReg(uint8 reg, uint8 *pVal)
{
    CS = CS_ENABLED;
    WAIT_1_3US(2);
    spiWriteByte(reg);
    spiReadByte(pVal, 0xFF);
    CS = CS_DISABLED;
}

/** \brief	Read x, y and z acceleration data
*
* Read x, y and z acceleration data in one operation.
* NOTE: No sensor access must be made immidiately folloing this operation
* without enabling the last Wait call.
*
* \param[in]       *pXVal
*     Pointer to variable to put read out X acceleration
* \param[in]       *pYVal
*     Pointer to variable to put read out Y acceleration
* \param[in]       *pZVal
*     Pointer to variable to put read out Z acceleration
*/

void accReadAcc(int8 *pXVal, int8 *pYVal, int8 *pZVal)
{
    // X
    accReadReg(DOUTX, (uint8*)pXVal);
    WAIT_1_3US(80);

    // Y
    accReadReg(DOUTY, (uint8*)pYVal);
    WAIT_1_3US(80);

    //Z
    accReadReg(DOUTZ, (uint8*)pZVal);

    //WAIT_1_3US(80);


}


/** \brief	Write one byte to SPI interface
*
* Write one byte to SPI interface
*
* \param[in]       write
*     Value to write
*/
void spiWriteByte(uint8 write)
{
        U0CSR &= ~0x02;                 // Clear TX_BYTE
        U0DBUF = write;
        while (!(U0CSR & 0x02));        // Wait for TX_BYTE to be set
}

/** \brief	Read one byte from SPI interface
*
* Read one byte from SPI interface
*
* \param[in]       read
*     Read out value
* \param[in]       write
*     Value to write
*/
void spiReadByte(uint8 *read, uint8 write)
{
        U0CSR &= ~0x02;                 // Clear TX_BYTE
        U0DBUF = write;
        while (!(U0CSR & 0x02));        // Wait for TX_BYTE to be set
        *read = U0DBUF;
}

