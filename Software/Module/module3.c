#include "module3.h"

void Module3_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct); 	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB,&GPIO_InitStruct); 
	
	GPIO_SetBits(GPIOA,GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_0| GPIO_Pin_10 | GPIO_Pin_11);
}

void Module3_Mode0_Handler(void)
{
	
}

void Module3_Mode1_Handler(void)
{
	
}

