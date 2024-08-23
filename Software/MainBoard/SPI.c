/**
 * File Name: SPI.c
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides SPI functions implementation
 */


/* Includes ------------------------------------------------------------------*/
#include "SPI.h"

// SPI1 Initialization
void SPI1_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
    
	// Configure MOSI (PA7) and SCK (PA5)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	// Configure MISO (PA6)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	// Configure NSS (PA4)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	SPI_InitTypeDef SPI_InitStructure;
    
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
    
	SPI_Cmd(SPI1, ENABLE);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
}

void SPI1_Start(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
}

void SPI1_Stop(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

uint8_t SPI1_SwapByte(uint8_t ByteSend)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET);
    
    SPI_I2S_SendData(SPI1, ByteSend);
    
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET);
    
    return SPI_I2S_ReceiveData(SPI1);
}

void SPI2_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure NSS (PA12)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	// Configure MOSI (PB15) and SCK (PB13)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	// Configure MISO (PA14)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	SPI_InitTypeDef SPI_InitStructure;					
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;				
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;			
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;				
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;				
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				
	SPI_InitStructure.SPI_CRCPolynomial = 7;				
	SPI_Init(SPI2, &SPI_InitStructure);					
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	SPI_Cmd(SPI2, ENABLE);	
}

void SPI2_Start(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
}

void SPI2_Stop(void)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

uint8_t SPI2_SwapByte(uint8_t ByteSend)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET);
	
	SPI_I2S_SendData(SPI2, ByteSend);
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) != SET);
	
	return SPI_I2S_ReceiveData(SPI2);
}