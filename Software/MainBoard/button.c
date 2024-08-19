/**
 * File Name: button.c
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides button functions implementation
 */
 
/* Includes ------------------------------------------------------------------*/
#include "button.h"
#include <stdio.h>

#define TIMER_FOR_BUTTON_ENABLE TIM_Cmd(TIM_FOR_BUTTON,ENABLE)
#define TIMER_FOR_BUTTON_DISABLE TIM_Cmd(TIM_FOR_BUTTON,DISABLE)

Button_t Button;
static volatile uint16_t time_release_count_ms = 0;
static volatile uint16_t time_hold_count_ms = 0;
static volatile uint8_t previous_status = 0;



/// @brief Button EXTI handler
/// @param  None
void EXTI0_IRQHandler(void)
{
        if(EXTI_GetITStatus(EXTI_Line0)==SET){
		//If previous button status is handled.Then button status must change to IDLE.
		if(Button.status == BUTTON_IDLE){
			TIMER_FOR_BUTTON_ENABLE;
		}
                EXTI_ClearITPendingBit(EXTI_Line0);
        }
}

/// @brief Button status update for a given time
/// @param  None
void TIM4_IRQHandler(void)
{
        if(TIM_GetITStatus(TIM_FOR_BUTTON,TIM_IT_Update) == SET){
		//Read Button status on GPIO for now
		uint8_t current_status = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
		//If status for now and previous is same.And button is held.
		if(previous_status == current_status && current_status == 0)
		{
			time_hold_count_ms++;
			//If button hold a long time,Stop.
			if(time_hold_count_ms >= 5000)
			{
				time_hold_count_ms = BUTTON_SHORT_LONG_THRESHOLD_MS;
			}
			
		}
		//If status for now and previous is same.And button is released.
		else if(previous_status == current_status && current_status == 1){
			time_release_count_ms++;
			
			if(time_hold_count_ms >= BUTTON_SHORT_LONG_THRESHOLD_MS){
					Button.status = BUTTON_HOLD_LONG;
			}
			//If button status is stable for BUTTON_IDLE_SHORT_THRESHOLD_MS 
			else if(time_hold_count_ms >= BUTTON_IDLE_SHORT_THRESHOLD_MS){
					Button.status = BUTTON_HOLD;
			}
			//If button status is stable for BUTTON_IDLE_SHORT_THRESHOLD_MS ,Button is released.
			if(time_release_count_ms >= BUTTON_IDLE_SHORT_THRESHOLD_MS){
				time_hold_count_ms = 0;
				time_release_count_ms = 0;
				TIMER_FOR_BUTTON_DISABLE;
			}
		}
		//Status update 
		previous_status = current_status;
		TIM_ClearITPendingBit(TIM_FOR_BUTTON,TIM_IT_Update);
	}
}

void Button_Status_Clear(void)
{
	Button.status = BUTTON_IDLE;
}

void Button_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_GPIO_FOR_BUTTON,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_TIM_FOR_BUTTON,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
        TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStruct;
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
	
	//Button Init****************************************************//   
	//Button working at BUTTON_GPIO(reffer to config.h)
        GPIO_InitStruct.GPIO_Pin = BUTTON_GPIO_PIN;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
        
        EXTI_InitStruct.EXTI_Line = EXTI_Line0;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
        
        NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
        NVIC_Init(&NVIC_InitStruct);

//TIM_FOR_BUTTON Init******************************************************//

	TIM_TimerBaseInitStruct.TIM_Prescaler = 3600 - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = 20 - 1;                   //1ms per interrupt
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM_FOR_BUTTON,&TIM_TimerBaseInitStruct);
        
        TIM_ClearFlag(TIM_FOR_BUTTON,TIM_FLAG_Update);
	TIM_ITConfig(TIM_FOR_BUTTON,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM_FOR_BUTTON_IRQN;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
	NVIC_Init(&NVIC_InitStruct);   
	
	printf("Button_Init\n");
	Button.status_clear = &Button_Status_Clear;
}
