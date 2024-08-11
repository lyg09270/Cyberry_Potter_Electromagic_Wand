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
//#define SYSTEM_MODE_DATA_COLLECT
#define SERIAL_DEBUG

// Define Serial baud rate
// #define BAUD_RATE 9600
#define BAUD_RATE (115200)
//#define BAUD_RATE (921600)

//Define ADC Module Detect
#define ADC_MAX_SAMPLES 10
#define ADC_MAX_TESTS 100
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



//Define LED
#define LED_GPIO GPIOA
#define LED_GPIO_PIN GPIO_Pin_1

//Define Button
#define RCC_TIM_FOR_BUTTON RCC_APB1Periph_TIM4
#define BUTTON_GPIO GPIOA
#define BUTTON_GPIO_PIN GPIO_Pin_0
#define TIM_FOR_BUTTON TIM4
#define TIM_FOR_BUTTON_IRQN TIM4_IRQn
#define TIM4_PER_INTERRUPT_MS (1)

//Define TIM2 
#define TIM2_RECORD_OVERTIME_US 15000
#define TIM2_FREQUENCY 500000		//433Mhz Transmitter won't work if TIM2_FREQUENCY below 13000hz
#define US_PER_TIMER2_COUNT (int)(1*1000000/TIM2_FREQUENCY)
#define TIM2_PRESCALE SYSTEM_FREQUENCY/TIM2_FREQUENCY

//Define Button 
#define BUTTON_IDLE_SHORT_THRESHOLD_MS (10)
#define BUTTON_SHORT_LONG_THRESHOLD_MS (1000)
#define BUTTON_LONG_VERYLONG_THRESHOLD_MS (2500)

//Define Signal 
#define SIGNAL_SEQUENCE_SET_LENGTH 512
#define SIGNAL_SEQUENCE_SET_BYTE_LENGTH 1024

//Define External ROM
#define W25Q64_PAGE_SIZE 256
#define W25Q64_SECTOR_ADDRESS_INCREMENT 0x0001000
#define W25Q64_SIGNAL_TYPE_INCREMENT 0x000C000

//Define some useful constant


//Define IMU
#define IMU_IS_MPU6050
#define IMU_DATA_PRINT_HEADER "IMU\n"
#define IMU_DEFAULT_HZ  (100)
#define IMU_SAMPLING_TIME_MS (1500)
#define IMU_SEQUENCE_LENGTH_MAX (150)
#define IMU_ACC_TRANS_CONSTANT (8192.0)  //+-4g
//+-500 degrees raw value to degree constant is divided by 16.384
//+-500 to radian is divided by (73.537*180/PI) = 4213.359738
#define IMU_GYRO_TRANS_RADIAN_CONSTANT (4213.359738) 

//Define CNN
//The out put of model must bigger than this value unless it will give a unrecognized message
#define OUPUT_THRESHOLD 103 

#endif  //_CONFIG_H_
