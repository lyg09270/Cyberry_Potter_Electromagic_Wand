/**************************************************************************************************
  Filename:       cma3000d.h
  Revised:        $Date: 2013-02-08 05:16:52 -0800 (Fri, 08 Feb 2013) $
  Revision:       $Revision: 33023 $

  Description:    Header file for control of the accelerometer on the keyfob
                  board in the CC2540DK-mini kit.

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

#ifndef CMA3000_H
#define CMA3000_H

#include "hal_types.h"


//***********************************************************************************
// Defines

// HW version
#define REV_1_0
#ifndef REV_1_0
  #define REV_0_6
#endif

// CMA3000 addressing space
#define WHO_AM_I        (0x00<<2)
#define REVID           (0x01<<2)
#define CTRL            (0x02<<2)
#define STATUS          (0x03<<2)
#define RSTR            (0x04<<2)
#define INT_STATUS      (0x05<<2)
#define DOUTX           (0x06<<2)
#define DOUTY           (0x07<<2)
#define DOUTZ           (0x08<<2)
#define MDTHR           (0x09<<2)
#define MDFFTMR         (0x0A<<2)
#define FFTHR           (0x0B<<2)

// CTRL register definitions
#define RANGE_2G        0x80
#define RANGE_8G        0x00

#define INT_ACTIVE_LOW  0x40
#define INT_ACTIVE_HIGH 0x00

#define MODE_PD         0x00
#define MODE_100HZ_MEAS 0x02
#define MODE_400HZ_MEAS 0x04
#define MODE_40HZ_MEAS  0x06
#define MODE_10HZ_MD    0x08
#define MODE_100HZ_FALL 0x0A
#define MODE_400HZ_FALL 0x0C

#define INT_DIS         0x01
#define INT_EN          0x00




// Data ready pin
#ifdef REV_1_0
    #define ACC_DATA_READY  (P1_7 == 1)
#elif (defined REV_0_6)
    #define ACC_DATA_READY  (P0_6 == 1)
#endif

//***********************************************************************************
// Macros

// Wait 1+1/3*t [us]
#define WAIT_1_3US(t)                   \
    do{                                 \
        for (uint8 i = 0; i<t; i++)     \
            asm("NOP");                 \
    }while(0)



//***********************************************************************************
// Function prototypes
void accInit(void);
void accStop(void);
void accWriteReg(uint8 reg, uint8 val);
void accReadReg(uint8 reg, uint8 *pVal);
void accReadAcc(int8 *pXVal, int8 *pYVal, int8 *pZVal);



#endif
