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
#include "LED.h"
#include "Delay.h"

#define LED_ON GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_SET)
#define LED_OFF GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_RESET)

LED_t LED;

void LED_Blink(eLED_STATUS status)
{
	int i = 0;
	
	switch(status)
	{
		case OFF:
			LED_OFF;
			break;
		case ON:
			LED_ON;
			break;
		case BLINK_5HZ:
			for(i = 0; i < 6; i++){
			LED_OFF;
			Delay_ms(100);
			LED_ON;    
			Delay_ms(100);		
			}
			break;
		case BLINK_10HZ:
			for(i = 0; i < 11; i++){
			LED_OFF;
			Delay_ms(50);
			LED_ON;    
			Delay_ms(50);		
			}
			break;
		case BLINK_2HZ:
			for(i = 0; i < 4; i++){
			LED_OFF;
			Delay_ms(250);
			LED_ON;    
			Delay_ms(250);		
			}
			break;	
	}
}

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(LED_GPIO,&GPIO_InitStruct); 	
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct); 	
	
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);
	
	LED.Operate = &LED_Blink;
	LED.Operate(ON);
}