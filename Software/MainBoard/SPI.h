/**
 * File Name: SPI.h
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides SPI functions implementation
 */


#ifndef	_SPI_H_
#define	_SPI_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"                  // Device header

/* Functions -----------------------------------------------------------------*/
void SPI1_Init(void);
void SPI1_Start(void);
void SPI1_Stop(void);
uint8_t SPI1_SwapByte(uint8_t ByteSend);

void SPI2_Init(void);
void SPI2_Start(void);
void SPI2_Stop(void);
uint8_t SPI2_SwapByte(uint8_t ByteSend);
#endif	//_SPI_H_
