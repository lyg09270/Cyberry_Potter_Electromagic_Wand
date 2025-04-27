/**
 * File Name: LED.h
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides LED functions implementation
 */


#ifndef	_LED_H_
#define	_LED_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"                  // Device header

typedef enum eLED_LED{
        OFF = 0,
	ON = 1,
	BLINK_2HZ = 2,
        BLINK_5HZ = 3,
        BLINK_10HZ = 4
}eLED_STATUS;

typedef struct LED_t
{
	void (*Operate)(eLED_STATUS status);
	eLED_STATUS status;
}LED_t;

//Define LED
#define LED_GPIO GPIOA
#define LED_GPIO_PIN GPIO_Pin_1
//#define LASER_ENABLE

extern struct LED_t LED;
#ifdef LASER_ENABLE
extern struct LED_t Laser;
#endif

/* Functions -----------------------------------------------------------------*/
void LED_Init(void);

#endif	//_LED_H_
