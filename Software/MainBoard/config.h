#ifndef _CONFIG_H_
#define _CONFIG_H_

// This file contains compile-time configurations for Cyberry Potter magical wand internal system.


#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Define running mode.
// NOTE: You can change the wand into different mode by the following defines.
//      When you change into another mode,you should comment the define of previous mode. 
// CYBERRY_POTTER_MODE_NORMAL: Normal mode.
// CYBERRY_POTTER_MODE_DATA_COLLECTOR: Running at data collecting mode to collect 
//      data from IMU and then send those data to serial port.Those data are used
//      to training motion identify model.

//Define system configuration
#define SYSTEM_FREQUENCY (72000000)
#define SERIAL_DEBUG
//#define SYSTEM_MODE_DATA_COLLECT

//Define ADC Module Detect
#define MODULE0_LOWER_LIM 186
#define MODULE0_UPPER_LIM 555
#define MODULE1_LOWER_LIM 555
#define MODULE1_UPPER_LIM 865
#define MODULE2_LOWER_LIM 865
#define MODULE2_UPPER_LIM 1166
#define MODULE3_LOWER_LIM 1166
#define MODULE3_UPPER_LIM 1460
#define MODULE4_LOWER_LIM 1460
#define MODULE4_UPPER_LIM 1815
#define MODULE5_LOWER_LIM 1815
#define MODULE5_UPPER_LIM 2281
#define MODULE6_LOWER_LIM 2281
#define MODULE6_UPPER_LIM 2636
#define MODULE7_LOWER_LIM 2636
#define MODULE7_UPPER_LIM 2930
#define MODULE8_LOWER_LIM 2930
#define MODULE8_UPPER_LIM 3231
#define MODULE9_LOWER_LIM 3231
#define MODULE9_UPPER_LIM 3541
#define MODULE10_LOWER_LIM 3541
#define MODULE10_UPPER_LIM 3910

//Define External ROM
#define W25Q64_PAGE_SIZE 256
#define W25Q64_SECTOR_ADDRESS_INCREMENT 0x0001000
#define W25Q64_SIGNAL_TYPE_INCREMENT 0x000C000

//Define some useful constant

//Define CNN
//The out put of model must bigger than this value unless it will give a unrecognized message

#endif  //_CONFIG_H_
