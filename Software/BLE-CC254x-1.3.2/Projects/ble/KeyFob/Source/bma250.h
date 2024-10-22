/**************************************************************************//**
    @file       bma250.h

    @brief      Header file for accelerometer BMA250. @Note This header file
                does not include all register addresses for the BMA250.

******************************************************************************/
#ifndef BMA250_H
#define BMA250_H


/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"


/******************************************************************************
 * DEFINES
 */

// BMA250 addressing space
#define ACC_CHIPID                  0x00    // Always 0x03
#define ACC_X_LSB                   0x02    // ACC_X_LSB[7:6] = 2 LSb of X acceleration data
#define ACC_X_MSB                   0x03    // ACC_X_MSB[7:0] = 8 MSb of X data
#define ACC_Y_LSB                   0x04
#define ACC_Y_MSB                   0x05
#define ACC_Z_LSB                   0x06
#define ACC_Z_MSB                   0x07
#define ACC_TEMP                    0x08    // Temperature data
#define ACC_INT_STATUS              0x09
#define ACC_DATA_INT_STATUS         0x0A
#define ACC_RANGE                   0x0F    // 2/4/8/16 G range
#define ACC_BW                      0x10    // Filtered bandwidth
#define ACC_PM                      0x11    // Susp[7], Low_power[6], sleep_dur[4:1]
#define ACC_CONF_FILT_SHADOW        0x13
#define ACC_SOFTRESET               0x14
#define ACC_INT_ENABLE0             0x16
#define ACC_INT_ENABLE1             0x17
#define ACC_INT_MAPPING0            0x19
#define ACC_INT_MAPPING1            0x1A
#define ACC_INT_MAPPING2            0x1B
#define ACC_INT_SOURCE              0x1E
#define ACC_INT_PIN_BEHAVIOR        0x20

// Range selection definitions
#define ACC_RANGE_2G                0x03    //  3.91 mg/LSB
#define ACC_RANGE_4G                0x05    //  7.81 mg/LSB
#define ACC_RANGE_8G                0x08    // 15.62 mg/LSB
#define ACC_RANGE_16G               0x0C    // 31.25 mg/LSB

// Filtered bandwidth selection (delta_t = time between successive acc samples)
#define ACC_BW_7_81HZ               0x08    // 7.81Hz bandwidth (delta_t = 64 ms)
#define ACC_BW_15_63HZ              0x09    // delta_t = 32   ms
#define ACC_BW_31_25HZ              0x0A    // delta_t = 16   ms
#define ACC_BW_62_5HZ               0x0B    // delta_t =  8   ms
#define ACC_BW_125HZ                0x0C    // delta_t =  4   ms
#define ACC_BW_250HZ                0x0D    // delta_t =  2   ms
#define ACC_BW_500HZ                0x0E    // delta_t =  1   ms
#define ACC_BW_1000HZ               0x0F    // delta_t =  0.5 ms

#define ACC_PM_SUSP                 0x80    // Power mode register (0x11), bit 7
#define ACC_PM_LP                   0x40    // Low power mode
#define ACC_PM_SLEEP_10MS           0x14
#define ACC_PM_SLEEP_25MS           0x16
#define ACC_PM_SLEEP_50MS           0x18

// Interrupt enable bitmasks (for use with registers ACC_INT_ENABLEx [x=0,1] )
#define ACC_INT_FLAT_EN             0x80    // Bit in register 0x16
#define ACC_INT_ORIENT_EN           0x40    //          "
#define ACC_INT_S_TAP_EN            0x20    //          "
#define ACC_INT_D_TAP_EN            0x10    //          "
#define ACC_INT_SLOPE_Z_EN          0x04    //          "
#define ACC_INT_SLOPE_Y_EN          0x02    //          "
#define ACC_INT_SLOPE_X_EN          0x01    //          "
#define ACC_INT_DATA_EN             0x10    // Bit in register 0x17
#define ACC_INT_LOW_EN              0x08    //          "
#define ACC_INT_HIGH_Z_EN           0x04    //          "
#define ACC_INT_HIGH_Y_EN           0x02    //          "
#define ACC_INT_HIGH_X_EN           0x01    //          "

// Interrupt mapping bitmasks (for use with registers ACC_INT_MAPPINGx [x=0,1,2] )
#define ACC_INT_MAP_FLAT            0x80    // For pin INT1 (INT2), bit in register 0x19 (0x1B)
#define ACC_INT_MAP_ORIENT          0x40    //                   "
#define ACC_INT_MAP_S_TAP           0x20    //                   "
#define ACC_INT_MAP_D_TAP           0x10    //                   "
#define ACC_INT_MAP_SLOPE           0x04    //                   "
#define ACC_INT_MAP_HIGH            0x02    //                   "
#define ACC_INT_MAP_LOW             0x01    //                   "
#define ACC_INT1_MAP_DATA           0x01    // New data IRQ to pin INT1, bit in register 0x1A
#define ACC_INT2_MAP_DATA           0x80    // New data IRQ to pin INT2, bit in register 0x1A

// Interrupt source bitmasks (for use with register ACC_INT_SOURCE)
#define ACC_INT_SRC_DATA_FILT       0x20
#define ACC_INT_SRC_TAP_FILT        0x01
#define ACC_INT_SRC_SLOPE_FILT      0x04
#define ACC_INT_SRC_HIGH_FILT       0x02
#define ACC_INT_SRC_LOW_FILT        0x01

// Interrupt pin behavior bitmasks (for use with register (Open drive/push-pull and active level 0/1)
#define ACC_INT2_OD                 0x08
#define ACC_INT2_LVL                0x04
#define ACC_INT1_OD                 0x02
#define ACC_INT1_LVL                0x01

// Perform soft reset
#define ACC_SOFTRESET_EN            0xB6    // Soft reset by writing 0xB6 to softreset register


/******************************************************************************
 * MACROS
 */
// Wait 1 [ms]
#define WAIT_1MS()      {for(unsigned short i=0;i<32000;i++)asm("NOP"); }

// Wait t [ms]
#define WAIT_MS(t)                      \
    do{                                 \
        for(uint8 i = 0; i<t; i++)      \
            WAIT_1MS();                 \
    }while(0)


/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void accInit(void);
void accStop(void);
void accWriteReg(uint8 reg, uint8 val);
void accReadReg(uint8 reg, uint8 *pVal);
void accReadAcc(int8 *pXVal, int8 *pYVal, int8 *pZVal);
void accReadAcc16(int16 *pXVal, int16 *pYVal, int16 *pZVal);


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


#endif
