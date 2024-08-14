/**
 * File Name: IIC.h
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides IIC functions implementation
 */


#ifndef	_IIC_H_
#define	_IIC_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"                  // Device header

/* Functions -----------------------------------------------------------------*/
void IIC1_Init(void);
int IIC1_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
int IIC1_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
void IIC2_Init(void);
int IIC2_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
int IIC2_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

#endif	//_IIC_H_
