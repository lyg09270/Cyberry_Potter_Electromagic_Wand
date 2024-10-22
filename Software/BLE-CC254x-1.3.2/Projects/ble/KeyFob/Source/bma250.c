/**************************************************************************//**

    @file       bma250.c

    @brief      Functions for accessing accelerometer BMA250 on CC2541 keyfob.

******************************************************************************/


/******************************************************************************
 * INCLUDES
 */
#include <ioCC2541.h>
#include "bma250.h"


/******************************************************************************
 * DEFINES
 */
// Accelerometer connected at (names in brackets are BMA250 names)
// P2_0 = VDD (VDD/VDDIO)
// P1_2 = CS_N (CSB)
// P1_3 = SCK (SCx)
// P1_4 = MISO (SDO)
// P1_5 = MOSI (SDx)
// P1_7 = (INT1)

#define CS              P1_2
#define SCK             P1_3
#define MISO            P1_4
#define MOSI            P1_5

#define CS_DISABLED     1
#define CS_ENABLED      0

// OR bitmask into reg (read, modify, write)
#define ACC_INT_ENABLE(reg, bm)     { uint8 val; accReadReg((reg),&val); \
                                      val |= (bm); accWriteReg((reg),val); }
// AND inverted bitmask into reg (read, modify write)
#define ACC_INT_DISABLE(reg, bm)    { uint8 val; accReadReg((reg),&val); \
                                      val &= ~(bm); accWriteReg((reg),val); }


/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void spiWriteByte(uint8 write);
void spiReadByte(uint8 *read, uint8 write);

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 acc_initialized = FALSE;


/******************************************************************************
 * FUNCTIONS
 */

/**************************************************************************//**
* @fn       accInit(void)
*
* @brief    Initialize SPI interface and BMA250 accelerometer.
*
* @return   void
******************************************************************************/
void accInit(void)
{
    //*** Enable Accelerometer power ***
    P2DIR |=  0x01;     // Set P2_0 as output high
    P2SEL &= ~0x01;
    P2    |=  0x01;

    //*** Setup USART 0 SPI at alternate location 2 ***
    // USART 0 at alternate location 2
    PERCFG |= 0x01;
    // Peripheral function on SCK, MISO and MOSI (P1_3-5)
    P1SEL |= 0x38;
    // Configure CS (P1_2) as output
    P1DIR |= 0x04;
    CS = CS_DISABLED;

    //*** Setup the SPI interface ***
    // SPI master mode
    U0CSR = 0x00;
    // Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
    //                                 data in on CPOL-inv -> CPOL
    // MSB first
    U0GCR = 0x20;
    // SCK frequency = 3MHz (MBA250 max=10MHz, CC254x max = 4MHz)
    U0GCR |= 0x10;
    U0BAUD = 0x80;

    // *** Configure accelerometer ***
    // Wait 2ms for accelerometer to power up and settle
    WAIT_MS(2);

    // Disable all interrupts
    ACC_INT_DISABLE(ACC_INT_ENABLE0, 0xFF);
    ACC_INT_DISABLE(ACC_INT_ENABLE1, 0xFF);

    // Set 2G range
    accWriteReg(ACC_RANGE, ACC_RANGE_2G);

    // Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
    accWriteReg(ACC_BW, ACC_BW_250HZ);


#ifdef ENABLE_BMA250_INTERRUPT
    // *** New data interrupt on INT1, active high ***
    // INT1 as push-pull, active high
    accWriteReg(ACC_INT_PIN_BEHAVIOR, ACC_INT1_LVL);

    // Map New data interrupt to INT1
    accWriteReg(ACC_INT_MAPPING1, ACC_INT1_MAP_DATA);

    // Enable new data interrupts
    ACC_INT_ENABLE(ACC_INT_ENABLE1, ACC_INT_DATA_EN);
#endif

    acc_initialized = TRUE;
} // accInit


/**************************************************************************//**
* @fn       accStop(void)
*
* @brief    Sets the BMA250 accelerometer in low-power mode.
*
* @return   void
******************************************************************************/
void accStop(void)
{
  if (acc_initialized) {
    // We cheat and simply turn off power to the accelerometer
    P2_0  =  0x00;  // If init has been run this pin is already output.
    acc_initialized = FALSE;
  }
}

/**************************************************************************//**
* @fn       accWriteReg(uint8 reg, uint8 val)
*
* @brief    Write one byte to a sensor register
*
* @param    reg     Register address
* @param    val     Value to write
*
* @return   void
******************************************************************************/
void accWriteReg(uint8 reg, uint8 val)
{
    CS = CS_ENABLED;
    spiWriteByte(reg);      // Write address
    spiWriteByte(val);      // Write value
    CS = CS_DISABLED;
}


/**************************************************************************//**
* @fn       accReadReg(uint8 reg, uint8 *pVal)
*
* @brief    Read one byte from a sensor register
*
* @param    reg     Register address
* @param    val     Pointer to destination of read value
*
* @return   void
******************************************************************************/
void accReadReg(uint8 reg, uint8 *pVal)
{
    CS = CS_ENABLED;
    spiWriteByte(0x80|reg);     // Write address
    spiReadByte(pVal, 0xFF);    // Write dummy data and read returned value
    CS = CS_DISABLED;
}


/**************************************************************************//**
* @fn       accReadAcc(int16 *pXVal, int16 *pYVal, int16 *pZVal)
*
* @brief    Read x, y and z acceleration data in one operation.
*
* @param    pXVal   Pointer to destination of read out X acceleration
* @param    pYVal   Pointer to destination of read out Y acceleration
* @param    pZVal   Pointer to destination of read out Z acceleration
*
* @return   void
******************************************************************************/
void accReadAcc(int8 *pXVal, int8 *pYVal, int8 *pZVal)
{
    int8 readout[6] = {0,0,0,0,0,0};

    // Read all data from accelerometer
    CS = CS_ENABLED;
    spiWriteByte(0x80|ACC_X_LSB);     // Write start address
    for(uint8 i = 0; i<6; i++)
    {
        spiReadByte((uint8 *)&readout[i], 0xFF); // Read byte
    }
    CS = CS_DISABLED;

    // Use only most significant byte of each channel.
    *pXVal = readout[1];
    *pYVal = readout[3];
    *pZVal = readout[5];

} // accReadAcc


/**************************************************************************//**
* @fn       accReadAcc(int16 *pXVal, int16 *pYVal, int16 *pZVal)
*
* @brief    Read x, y and z acceleration data in one operation.
*
* @param    pXVal   Pointer to destination of read out X acceleration
* @param    pYVal   Pointer to destination of read out Y acceleration
* @param    pZVal   Pointer to destination of read out Z acceleration
*
* @return   void
******************************************************************************/
void accReadAcc16(int16 *pXVal, int16 *pYVal, int16 *pZVal)
{
    int8 readout[6] = {0,0,0,0,0,0};

    // Read all data from accelerometer
    CS = CS_ENABLED;
    spiWriteByte(0x80|ACC_X_LSB);     // Write start address
    for(uint8 i = 0; i<6; i++)
    {
        spiReadByte((uint8 *)&readout[i], 0xFF); // Read byte
    }
    CS = CS_DISABLED;

    // Merge high byte (8b) and low bits (2b) into 16b signed destination
    *pXVal = ( (((uint8)readout[0]) >> 6) | ((int16)(readout[1]) << 2) );
    *pYVal = ( (((uint8)readout[2]) >> 6) | ((int16)(readout[3]) << 2) );
    *pZVal = ( (((uint8)readout[4]) >> 6) | ((int16)(readout[5]) << 2) );

} // accReadAcc16


/**************************************************************************//**
* @fn       spiWriteByte(uint8 write)
*
* @brief    Write one byte to SPI interface
*
* @param    write   Value to write
******************************************************************************/
void spiWriteByte(uint8 write)
{
        U0CSR &= ~0x02;                 // Clear TX_BYTE
        U0DBUF = write;
        while (!(U0CSR & 0x02));        // Wait for TX_BYTE to be set
}


/**************************************************************************//**
* @fn       spiReadByte(uint8 *read, uint8 write)
*
* @brief    Read one byte from SPI interface
*
* @param    read    Read out value
* @param    write   Value to write
******************************************************************************/
void spiReadByte(uint8 *read, uint8 write)
{
        U0CSR &= ~0x02;                 // Clear TX_BYTE
        U0DBUF = write;                 // Write address to accelerometer
        while (!(U0CSR & 0x02));        // Wait for TX_BYTE to be set
        *read = U0DBUF;                 // Save returned value
}


/******************************************************************************
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
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/
