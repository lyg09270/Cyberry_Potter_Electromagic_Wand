/**************************************************************************************************
  Filename:       buzzer.c
  Revised:        $Date: 2012-10-11 12:11:30 -0700 (Thu, 11 Oct 2012) $
  Revision:       $Revision: 31784 $

  Description:    Control of the buzzer of the keyfob board in the CC2540DK-mini
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

#include "hal_mcu.h"
#include "buzzer.h"


/** \brief	Initialize buzzer
*
* This will initialize the buzzer
*
*/
void buzzerInit(void)
{
#if defined ( CC2540_MINIDK )
    // Buzzer connected at P1_6
    // We will use Timer 3 Channel 0 at alternate location 2
    // Channel 0 will toggle on compare with 0 and counter will
    // count in up/down mode to T3CC0.

    PERCFG |= 0x20;             // Timer 3 Alternate location 2
    P1DIR |= 0x40;              // P1_6 = output
    P1SEL |= 0x40;              // Peripheral function on P1_6

    T3CTL &= ~0x10;             // Stop timer 3 (if it was running)
    T3CTL |= 0x04;              // Clear timer 3
    T3CTL &= ~0x08;             // Disable Timer 3 overflow interrupts
    T3CTL |= 0x03;              // Timer 3 mode = 3 - Up/Down

    T3CCTL0 &= ~0x40;           // Disable channel 0 interrupts
    T3CCTL0 |= 0x04;            // Ch0 mode = compare
    T3CCTL0 |= 0x10;            // Ch0 output compare mode = toggle on compare
#endif
}

/** \brief	Starts the buzzer
*
* Starts the buzzer with given frequency
*
* \param[in]       frequency
*     The frequency in Hertz of the sound to output
* @return  1 successful - 0 if frequency invalid
*/
uint8 buzzerStart(uint16 frequency)
{
#if defined ( CC2540_MINIDK )
    buzzerInit();
  
    uint8 prescaler = 0;

    // Get current Timer tick divisor setting
    uint8 tickSpdDiv = (CLKCONSTA & 0x38)>>3;

    // Check if frequency too low
    if (frequency < (244 >> tickSpdDiv)){   // 244 Hz = 32MHz / 256 (8bit counter) / 4 (up/down counter and toggle on compare) / 128 (max timer prescaler)
        buzzerStop();                       // A lower tick speed will lower this number accordingly.
        return 0;
    }

    // Calculate nr of ticks required to achieve target frequency
    uint32 ticks = (8000000/frequency) >> tickSpdDiv;      // 8000000 = 32M / 4;

    // Fit this into an 8bit counter using the timer prescaler
    while ((ticks & 0xFFFFFF00) != 0)
    {
        ticks >>= 1;
        prescaler += 32;
    }

    // Update registers
    T3CTL &= ~0xE0;
    T3CTL |= prescaler;
    T3CC0 = (uint8)ticks;

    // Start timer
    T3CTL |= 0x10;
#endif
    
    return 1;
}

/** \brief	Stops the buzzer
*
* Turn off the buzzer
*
*/
void buzzerStop(void)
{
#if defined ( CC2540_MINIDK )
    T3CTL &= ~0x10;             // Stop timer 3
    P1SEL &= ~0x40;
    P1_6 = 0;
#endif
}
