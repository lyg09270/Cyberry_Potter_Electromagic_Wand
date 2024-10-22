/**
 * File Name: ADC.h
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides ADC functions implementation
 */


#ifndef	_ADC_H_
#define	_ADC_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"    // Device header


#define ADC_MAX_SAMPLES 10
#define ADC_MAX_TESTS 10
#define SERIAL_DEBUG

/* Functions -----------------------------------------------------------------*/
void ADC_PB1_Init(void);
void ADC1_Deinit(void);
int8_t ADC_PB1_GetAvg(void);

#endif	//_ADC_H_
